#include "UBPlanner.h"

#include <QFile>
#include <QDebug>
#include <QLineF>
#include <QElapsedTimer>
#include <QGeoCoordinate>
#include <QCoreApplication>

#include "UBConfig.h"

#include "Waypoint.h"

#include <ilcplex/ilocplex.h>

ILOSTLBEGIN

UBPlanner::UBPlanner(QObject *parent) : QObject(parent),
    m_file(""),
    m_res(10),
    m_limit(1000000000),
    m_gap(0.01),
    m_lambda(1),
    m_gamma(1),
    m_kappa(1000000000),
    m_pcn(100)
{
    m_areas.clear();
    m_nodes.clear();
    m_agents.clear();
    m_depots.clear();

    m_agent_paths.clear();
}

QList<Waypoint*> UBPlanner::loadWaypoints(const QString &loadFile) {
    QList<Waypoint*> wps;

    QFile file(loadFile);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << QObject::tr("Not able to open the mission file!");
        return wps;
    }

    QTextStream in(&file);

    const QStringList &version = in.readLine().split(" ");

    if (version.length() < 2) {
        qWarning() << QObject::tr("Waypoint file is corrupt. Version not detectable");
        return wps;
    }

    int versionInt = version[2].toInt();

    if (!(version.size() == 3 && version[0] == "QGC" && version[1] == "WPL" && versionInt >= 110)) {
        qWarning() << QObject::tr("The waypoint file is version %1 and is not compatible").arg(versionInt);
    } else {
        int id = 0;
        while (!in.atEnd()) {
            Waypoint *t = new Waypoint();
            if(t->load(in)) {
                t->setId(id);
                wps.append(t);
                id++;
            } else {
                qWarning() << QObject::tr("The waypoint file is corrupted. Load operation only partly succesful.");
                break;
            }
        }
    }

    file.close();

    return wps;
}

bool UBPlanner::storeWaypoints(const QString& storeFile, QList<Waypoint*>& wps) {
    QFile file(storeFile);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qWarning() << QObject::tr("Not able to open the mission file!");
        return false;
    }

    QTextStream out(&file);

    //write the waypoint list version to the first line for compatibility check
    out << "QGC WPL 110\r\n";

    for (int i = 0; i < wps.count(); i++) {
        wps[i]->setId(i);
        wps[i]->save(out);
    }

    file.close();

    return true;
}

void UBPlanner::startPlanner() {
    QList<Waypoint*> wps = loadWaypoints(m_file);
    QList<Waypoint*>::const_iterator i = wps.begin();
    while (i != wps.end()) {
        if ((*i)->getAction() == MAV_CMD_NAV_TAKEOFF) {

            QPolygonF area;
            area << QPointF((*i)->getLatitude(), (*i)->getLongitude());

            QList<Waypoint*>::const_iterator j = i;
            while (j != wps.end()) {
                j++;

                area << QPointF((*j)->getLatitude(), (*j)->getLongitude());

                if ((*j)->getAction() == MAV_CMD_NAV_LAND) {
                    i = j;

                    area << area[0];
                    m_areas << area;

                    break;
                }
            }
        } else if ((*i)->getAction() == MAV_CMD_NAV_RETURN_TO_LAUNCH) {
            m_depots << 0;
            m_agents << QGeoCoordinate((*i)->getLatitude(), (*i)->getLongitude());
            m_agent_paths << QVector<QPair<quint32, quint32> >();
        }

        i++;
    }

    QElapsedTimer total_time;
    total_time.start();

    decompose();

    if (!divide()) {
        cerr << "Unable to divide the area between agents!" << endl;
        exit(EXIT_FAILURE);
    }

    QElapsedTimer agent_time;
    for (int a = 0; a < m_agents.size(); a++) {
        agent_time.restart();
        if (!planAgent(a) || !validatePath(a)) {
            cerr << "Unable to plan the coverage path for agent: " << a <<endl;
            exit(EXIT_FAILURE);
        }

        cout << "Elapsed time for agent " << a << " is "<< agent_time.elapsed() / 1000.0 << endl;

        buildMission(a);
    }

    emit planReady();

    cout << "The planner has successfully planned the mission for each agent in total time " << total_time.elapsed() / 1000.0 << endl;
    exit(EXIT_SUCCESS);
}

bool UBPlanner::decompose() {
    QPointF sf = m_areas[0].boundingRect().bottomLeft();
    QPointF rf = m_areas[0].boundingRect().bottomRight();
    QPointF uf = m_areas[0].boundingRect().topLeft();

    QGeoCoordinate s(sf.x(), sf.y());
    QGeoCoordinate r(rf.x(), rf.y());
    QGeoCoordinate u(uf.x(), uf.y());

    qreal xazimuth = s.azimuthTo(r);
    qreal yazimuth = s.azimuthTo(u);

    qreal xstep = ceil(s.distanceTo(r) / m_res);
    qreal ystep = ceil(s.distanceTo(u) / m_res);
    for (int j = 0; j < ystep; j++) {
        for (int i = 0; i < xstep; i++) {
            QGeoCoordinate x0 = s.atDistanceAndAzimuth(i * m_res, xazimuth);
            QGeoCoordinate y0 = s.atDistanceAndAzimuth(j * m_res, yazimuth);
            QGeoCoordinate x1 = s.atDistanceAndAzimuth(i * m_res, xazimuth);
            QGeoCoordinate y1 = s.atDistanceAndAzimuth((j + 1) * m_res, yazimuth);
            QGeoCoordinate x2 = s.atDistanceAndAzimuth((i + 1) * m_res, xazimuth);
            QGeoCoordinate y2 = s.atDistanceAndAzimuth((j + 1) * m_res, yazimuth);
            QGeoCoordinate x3 = s.atDistanceAndAzimuth((i + 1) * m_res, xazimuth);
            QGeoCoordinate y3 = s.atDistanceAndAzimuth(j * m_res, yazimuth);

            QPointF xf0(x0.latitude(), x0.longitude());
            QPointF yf0(y0.latitude(), y0.longitude());
            QPointF xf1(x1.latitude(), x1.longitude());
            QPointF yf1(y1.latitude(), y1.longitude());
            QPointF xf2(x2.latitude(), x2.longitude());
            QPointF yf2(y2.latitude(), y2.longitude());
            QPointF xf3(x3.latitude(), x3.longitude());
            QPointF yf3(y3.latitude(), y3.longitude());

            QVector<QPointF> cell;
            cell << xf0 + yf0 - sf << xf1 + yf1 - sf << xf2 + yf2 - sf << xf3 + yf3 - sf << xf0 + yf0 - sf;

            if (evaluate(cell)) {
                QGeoCoordinate xm = s.atDistanceAndAzimuth((i + 0.5) * m_res, xazimuth);
                QGeoCoordinate ym = s.atDistanceAndAzimuth((j + 0.5) * m_res, yazimuth);

                m_nodes << QGeoCoordinate(xm.latitude() + ym.latitude() - s.latitude(), xm.longitude() + ym.longitude() - s.longitude());
            }
        }
    }

    return true;
}

bool UBPlanner::evaluate(const QVector<QPointF>& cell) {
    for (int i = 0; i < cell.size() - 1; i++) {
        if (!m_areas[0].containsPoint(cell[i], Qt::OddEvenFill)) {
            return false;
        }

        for (int j = 1; j < m_areas.size(); j++) {
            if (m_areas[j].containsPoint(cell[i], Qt::OddEvenFill)) {
                return false;
            }
        }

        QLineF line(cell[i], cell[i + 1]);

        foreach (QPolygonF area, m_areas) {
            for (int k = 0; k < area.size() - 1; k++) {
                if (line.intersect(QLineF(area[k], area[k + 1]), NULL) == QLineF::BoundedIntersection) {
                    return false;
                }
            }
        }
    }

    return true;
}

bool UBPlanner::divide() {
    bool result = false;

    IloEnv env;

    IloNumArray2 dist_agent_node(env);
    for (int a = 0; a < m_agents.size(); a++) {
        dist_agent_node.add(IloNumArray(env, m_nodes.size()));
    }

    for (int a = 0; a < m_agents.size(); a++) {
        for (int i = 0; i < m_nodes.size(); i++) {
            dist_agent_node[a][i] = m_pcn * m_agents[a].distanceTo(m_nodes[i]);
        }
    }

    IloArray<IloBoolVarArray> x_agent_node(env);
    for (int a = 0; a < m_agents.size(); a++) {
        x_agent_node.add(IloBoolVarArray(env, m_nodes.size()));
    }

    IloFloatVar z(env);

    try {
        IloModel mod(env);
        mod.add(IloMinimize(env, z));

        for (int a = 0; a < m_agents.size(); a++) {
            IloExpr total_dist(env);
            for (int i = 0; i < m_nodes.size(); i++) {
                total_dist += dist_agent_node[a][i] * x_agent_node[a][i];
            }

            mod.add(total_dist <= z);

            total_dist.end();
        }

        for (int i = 0; i < m_nodes.size(); i++) {
            IloExpr flow_in(env);
            for (int a = 0; a < m_agents.size(); a++) {
                flow_in += x_agent_node[a][i];
            }

            mod.add(flow_in == 1);

            flow_in.end();
        }

        IloCplex cplex(mod);
//        cplex.exportModel("div.lp");
        if (!cplex.solve()) {
            throw(-1);
        }

        result = true;

        env.out() << "Minimume Cost = " << cplex.getObjValue() / m_pcn << endl;

        qreal min_dist = m_kappa;
        for (int a = 0; a < m_agents.size(); a++) {
            for (int i = 0; i < m_nodes.size(); i++) {
                if (cplex.getValue(x_agent_node[a][i])) {
                    m_agent_paths[a] << QPair<quint32, quint32>(i, i);

                    qreal dist = m_agents[a].distanceTo(m_nodes[i]);
                    if (dist < min_dist) {
                        min_dist = dist;
                        m_depots[a] = i;
                    }
                }
            }
        }
    }
    catch (IloException& e) {
        cerr << "Optimization Exception Caught: " << e << endl;
    }
    catch (...) {
        cerr << "Unable to divide the area beween agents!" << endl;
    }

    env.end();

    return result;
}

bool UBPlanner::planAgent(quint32 agent) {
    bool result = false;

    IloEnv env;

    IloIntArray2 dist_node_node(env);
    for (int i = 0; i < m_agent_paths[agent].size(); i++) {
        dist_node_node.add(IloIntArray(env, m_agent_paths[agent].size()));
    }

    IloIntArray3 turn_node_node_node(env);
    for (int i = 0; i < m_agent_paths[agent].size(); i++) {
        IloIntArray2 turn_node_node(env);
        for (int j = 0; j < m_agent_paths[agent].size(); j++) {
            turn_node_node.add(IloIntArray(env, m_agent_paths[agent].size()));
        }

        turn_node_node_node.add(turn_node_node);
    }

    // sqrt(2) < 1.5 < 2
    qreal max_dist = 1.5 * m_res;
    for (int i = 0; i < m_agent_paths[agent].size(); i++) {
        for (int j = 0; j < m_agent_paths[agent].size(); j++) {
            qreal dist = m_nodes[m_agent_paths[agent][i].first].distanceTo(m_nodes[m_agent_paths[agent][j].first]);
            if (!dist || dist > max_dist) {
                dist_node_node[i][j] = m_kappa;
            } else {
                dist_node_node[i][j] = m_pcn * dist;
            }
        }
    }

    for (int i = 0; i < m_agent_paths[agent].size(); i++) {
        for (int j = 0; j < m_agent_paths[agent].size(); j++) {
            for (int k = 0; k < m_agent_paths[agent].size(); k++) {
                if (dist_node_node[i][j] > m_pcn * max_dist || dist_node_node[j][k] > m_pcn * max_dist) {
                    turn_node_node_node[i][j][k] = m_kappa;
                } else {
                    qreal r = m_nodes[m_agent_paths[agent][i].first].distanceTo(m_nodes[m_agent_paths[agent][j].first]);
                    qreal e = m_nodes[m_agent_paths[agent][j].first].distanceTo(m_nodes[m_agent_paths[agent][k].first]);
                    qreal s = m_nodes[m_agent_paths[agent][k].first].distanceTo(m_nodes[m_agent_paths[agent][i].first]);
                    qreal t = (r * r + e * e - s * s) / (2.0 * r * e);
                    if (t > 1.0) {
                        t = 1.0;
                    } else if (t < -1.0) {
                        t = -1.0;
                    }

                    qreal turn = M_PI - acos(t);

                    turn_node_node_node[i][j][k] = m_pcn * turn;
                }
            }
        }
    }

    IloArray<IloBoolVarArray> x_node_node(env);
    for (int i = 0; i < m_agent_paths[agent].size(); i++) {
        x_node_node.add(IloBoolVarArray(env, m_agent_paths[agent].size()));
    }

    IloNumVarArray u(env, m_agent_paths[agent].size(), 0.0, IloInfinity, ILOFLOAT);

    try {
        IloModel mod(env);

        IloExpr total_dist(env);
        for (int i = 0; i < m_agent_paths[agent].size(); i++) {
            for (int j = 0; j < m_agent_paths[agent].size(); j++) {
                if (j == i) {
                    continue;
                }

                total_dist += dist_node_node[i][j] * x_node_node[i][j];
            }
        }

        IloExpr total_turn(env);
        for (int i = 0; i < m_agent_paths[agent].size(); i++) {
            for (int j = 0; j < m_agent_paths[agent].size(); j++) {
                if (j == i || m_agent_paths[agent][j].first == m_depots[agent]) {
                    continue;
                }

                for (int k = 0; k < m_agent_paths[agent].size(); k++) {
                    if (k == j) {
                        continue;
                    }

                    total_turn += turn_node_node_node[i][j][k] * x_node_node[i][j] * x_node_node[j][k];
                }
            }
        }

        mod.add(IloMinimize(env, m_lambda * total_dist + m_gamma * total_turn));

        total_dist.end();
        total_turn.end();

        for (int j = 0; j < m_agent_paths[agent].size(); j++) {
            IloExpr flow_in(env);
            for (int i = 0; i < m_agent_paths[agent].size(); i++) {
                if (i == j) {
                    continue;
                }

                flow_in += x_node_node[i][j];
            }

            mod.add(flow_in == 1);

            flow_in.end();
        }

        for (int i = 0; i < m_agent_paths[agent].size(); i++) {
            IloExpr flow_out(env);
            for (int j = 0; j < m_agent_paths[agent].size(); j++) {
                if (j == i) {
                    continue;
                }

                flow_out += x_node_node[i][j];
            }

            mod.add(flow_out == 1);

            flow_out.end();
        }

        for (int i = 0; i < m_agent_paths[agent].size(); i++) {
            if (m_agent_paths[agent][i].first == m_depots[agent]) {
                continue;
            }

            for (int j = 0; j < m_agent_paths[agent].size(); j++) {
                if (m_agent_paths[agent][j].first == m_depots[agent] || j == i) {
                    continue;
                }

                mod.add(u[i] - u[j] + m_agent_paths[agent].size() * x_node_node[i][j] <= m_agent_paths[agent].size() - 1);
            }
        }

        IloCplex cplex(mod);
        cplex.setParam(IloCplex::EpGap, m_gap);
        cplex.setParam(IloCplex::TiLim, m_limit);
        if (!cplex.solve() || cplex.getObjValue() / m_pcn >= m_kappa) {
            throw(-1);
        }

        result = true;

        env.out() << "Minimume Cost = " << cplex.getObjValue() / m_pcn << endl;

        for (int i = 0; i < m_agent_paths[agent].size(); i++) {
            for (int j = 0; j < m_agent_paths[agent].size(); j++) {
                if (j == i) {
                    continue;
                }

                if (cplex.getValue(x_node_node[i][j])) {
                    m_agent_paths[agent][i].second = m_agent_paths[agent][j].first;

                    break;
                }
            }
        }
    }
    catch (IloException& e) {
        cerr << "Optimization Exception Caught: " << e << endl;
    }
    catch (...) {
        cerr << "Unable to plan a path for agent: " << agent << endl;
    }

    env.end();

    return result;
}

bool UBPlanner::validatePath(quint32 agent) {
    qreal total_dist = 0;
    qreal total_turn = 0;

    quint32 ang1 = 0;
    quint32 ang2 = 0;
    quint32 ang3 = 0;

    quint32 i = m_depots[agent];
    quint32 j = m_depots[agent];
    quint32 k = m_depots[agent];

    for (int node = 0; node < m_agent_paths[agent].size(); node++) {
        if (m_agent_paths[agent][node].first == i) {
            j = m_agent_paths[agent][node].second;

            break;
        }
    }

    // sqrt(2) < 1.5 < 2
    qreal max_dist = 1.5 * m_res;
    while (true) {
        qreal dist = m_nodes[i].distanceTo(m_nodes[j]);
        if (dist > max_dist) {
            return false;
        }

        total_dist += dist;

        if (j == m_depots[agent]) {
            break;
        }

        for (int node = 0; node < m_agent_paths[agent].size(); node++) {
            if (m_agent_paths[agent][node].first == j) {
                k = m_agent_paths[agent][node].second;

                break;
            }
        }

        qreal r = m_nodes[i].distanceTo(m_nodes[j]);
        qreal e = m_nodes[j].distanceTo(m_nodes[k]);
        qreal s = m_nodes[k].distanceTo(m_nodes[i]);
        qreal t = (r * r + e * e - s * s) / (2.0 * r * e);
        if (t > 1.0) {
            t = 1.0;
        } else if (t < -1.0) {
            t = -1.0;
        }

        qreal turn = M_PI - acos(t);

        total_turn += turn;

        if (turn > M_PI / 4.0 - M_PI / 8.0 && turn < M_PI / 4.0 + M_PI / 8.0) {
            ang1++;
        } else if (turn > M_PI / 2.0 - M_PI / 8.0 && turn < M_PI / 2.0 + M_PI / 8.0) {
            ang2++;
        } else if (turn > 3.0 * M_PI / 4.0 - M_PI / 8.0 && turn < 3.0 * M_PI / 4.0 + M_PI / 8.0) {
            ang3++;
        }

        i = j;
        j = k;
    }

    cout << "Total Distance: " << total_dist << " | Number of 45' Turn: " << ang1 << " | Number of 90' Turn: " << ang2 << " | Number of 135' Turn: " << ang3 << endl;
    cout << "Total Cost: " << m_lambda * total_dist + m_gamma * total_turn << endl;

    return true;
}

bool UBPlanner::buildMission(quint32 agent) {
    QList<Waypoint*> wps;

    Waypoint* wp = new Waypoint();
//    wp.setFrame(MAV_FRAME_GLOBAL_RELATIVE_ALT);
    wp->setAcceptanceRadius(POINT_ZONE);
    wp->setLatitude(m_nodes[m_depots[agent]].latitude());
    wp->setLongitude(m_nodes[m_depots[agent]].longitude());
    wps.append(wp);

    wp = new Waypoint();
    wp->setAction(MAV_CMD_NAV_TAKEOFF);
    wp->setAcceptanceRadius(POINT_ZONE);
    wp->setLatitude(m_nodes[m_depots[agent]].latitude());
    wp->setLongitude(m_nodes[m_depots[agent]].longitude());
    wp->setAltitude(TAKEOFF_ALT);
    wps.append(wp);

    quint32 node = m_depots[agent];
    while (true) {
        for (int i = 0; i < m_agent_paths[agent].size(); i++) {
            if (m_agent_paths[agent][i].first == node) {
                node = m_agent_paths[agent][i].second;

                break;
            }
        }

        wp = new Waypoint();
        wp->setAction(MAV_CMD_NAV_WAYPOINT);
        wp->setAcceptanceRadius(POINT_ZONE);
        wp->setLatitude(m_nodes[node].latitude());
        wp->setLongitude(m_nodes[node].longitude());
        wp->setAltitude(TAKEOFF_ALT);
        wps.append(wp);

        if (node == m_depots[agent]) {
            break;
        }
    }

    wp = new Waypoint();
    wp->setAction(MAV_CMD_NAV_LAND);
    wp->setAcceptanceRadius(POINT_ZONE);
    wp->setLatitude(m_nodes[node].latitude());
    wp->setLongitude(m_nodes[node].longitude());
    wps.append(wp);

    if (!storeWaypoints(tr("mission_%1.txt").arg(agent), wps)) {
        return false;
    }

    return true;
}
