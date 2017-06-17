#include "UBPlanner.h"

#include <QLineF>
#include <QElapsedTimer>
#include <QCoreApplication>

#include "config.h"

#include "QsLog.h"
#include "UASWaypointManager.h"
#include "mercatorprojection.h"

#include <ilcplex/ilocplex.h>

ILOSTLBEGIN

UBPlanner::UBPlanner(QObject *parent) : QObject(parent),
    m_lambda(1),
    m_gamma(1),
    m_kappa(1000000000),
    m_dim(10),
    m_gap(0.01),
    m_pcs(100),
    m_limit(1000000000)
{
    m_areas.clear();
    m_nodes.clear();
    m_agents.clear();
    m_depots.clear();

    m_agent_paths.clear();
}

void UBPlanner::startPlanner() {
    if (QCoreApplication::arguments().contains("--help") || !QCoreApplication::arguments().contains("--area")) {
        cout << "ub-anc-planner --area filename [options]\n"
             << "Options:\n"
             << "\t --area \t\t filename of the area\n"
             << "\n"
             << "\t --res \t\t resolution of the area decomposition\n"
             << "\t --gap \t\t gap to the optimal solution\n"
             << "\t --lim \t\t optimizer time limit in seconds\n"
             << "\t --dis \t\t distance factor\n"
             << "\t --dir \t\t direction factor\n"
             << "\t --max \t\t maximum capacity for each drone\n"
             << "\n"
             << "\t --help \t\t show this message\n";
        exit(EXIT_SUCCESS);
    }

    int idx = QCoreApplication::arguments().indexOf("--res");
    if (idx > 0) {
        m_dim = QCoreApplication::arguments().at(idx + 1).toDouble();
    }

    idx = QCoreApplication::arguments().indexOf("--gap");
    if (idx > 0) {
        m_gap = QCoreApplication::arguments().at(idx + 1).toDouble();
    }

    idx = QCoreApplication::arguments().indexOf("--lim");
    if (idx > 0) {
        m_limit = QCoreApplication::arguments().at(idx + 1).toDouble();
    }

    idx = QCoreApplication::arguments().indexOf("--dis");
    if (idx > 0) {
        m_lambda = QCoreApplication::arguments().at(idx + 1).toDouble();
    }

    idx = QCoreApplication::arguments().indexOf("--dir");
    if (idx > 0) {
        m_gamma = QCoreApplication::arguments().at(idx + 1).toDouble();
    }

    idx = QCoreApplication::arguments().indexOf("--max");
    if (idx > 0) {
        m_kappa = QCoreApplication::arguments().at(idx + 1).toDouble();
    }

    UASWaypointManager wpm;
    wpm.loadWaypoints(QCoreApplication::arguments().at(QCoreApplication::arguments().indexOf("--area") + 1));

    core::Point pix;
    projections::MercatorProjection proj;

    QList<Waypoint*> wps = wpm.getWaypointEditableList();
    QList<Waypoint*>::const_iterator i = wps.begin();
    m_res = proj.GetGroundResolution(GND_RES, (*i)->getLatitude());
    while (i != wps.end()) {
        if ((*i)->getAction() == MAV_CMD_NAV_TAKEOFF) {
            pix = proj.FromLatLngToPixel((*i)->getLatitude(), (*i)->getLongitude(), GND_RES);

            QPolygonF area;
            area << QPointF(pix.X(), pix.Y());

            QList<Waypoint*>::const_iterator j = i;
            while (j != wps.end()) {
                j++;

                pix = proj.FromLatLngToPixel((*j)->getLatitude(), (*j)->getLongitude(), GND_RES);

                area << QPointF(pix.X(), pix.Y());

                if ((*j)->getAction() == MAV_CMD_NAV_LAND) {
                    i = j;

                    area << area[0];
                    m_areas << area;

                    break;
                }
            }
        } else if ((*i)->getAction() == MAV_CMD_NAV_RETURN_TO_LAUNCH) {
            pix = proj.FromLatLngToPixel((*i)->getLatitude(), (*i)->getLongitude(), GND_RES);

            m_depots << 0;
            m_agents << QPointF(pix.X(), pix.Y());
            m_agent_paths << QVector<QPair<quint32, quint32> >();
        }

        i++;
    }

    plan();
}

void UBPlanner::decompose() {
    qreal xmin = m_areas[0].boundingRect().topLeft().x();
    qreal ymin = m_areas[0].boundingRect().topLeft().y();

    qreal xmax = m_areas[0].boundingRect().bottomRight().x();
    qreal ymax = m_areas[0].boundingRect().bottomRight().y();

    int x = 0;
    int y = 0;
    while (true) {
        QPointF vtx(xmin + x * m_dim / m_res, ymin + y * m_dim / m_res);

        if (vtx.x() > xmax) {
            x = 0;
            y++;

            vtx.setX(xmin + x * m_dim / m_res);
            vtx.setY(ymin + y * m_dim / m_res);

            if (vtx.y() > ymax) {
                break;
            }
        }

        QVector<QPointF> cell;
        cell << vtx << vtx + QPointF(m_dim / m_res, 0) << vtx + QPointF(m_dim / m_res, m_dim / m_res) << vtx + QPointF(0, m_dim / m_res) << vtx;

        if (evaluate(cell)) {
            m_nodes << vtx + QPointF(m_dim / (2.0 * m_res), m_dim / (2.0 * m_res));
        }

        x++;
    }
}

bool UBPlanner::evaluate(const QVector<QPointF>& cell) {
    for (int i = 0; i < cell.size() - 1; i++) {
        if (!m_areas[0].containsPoint(cell[i], Qt::OddEvenFill))
            return false;

        for (int i = 1; i < m_areas.size(); i++)
            if (m_areas[i].containsPoint(cell[i], Qt::OddEvenFill))
                return false;

        QLineF line(cell[i], cell[i + 1]);

        foreach (QPolygonF area, m_areas)
            for (int i = 0; i < area.size() - 1; i++)
                if (line.intersect(QLineF(area[i], area[i + 1]), NULL) == QLineF::BoundedIntersection)
                    return false;
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
            dist_agent_node[a][i] = sqrt(pow(m_agents[a].x() - m_nodes[i].x(), 2) + pow(m_agents[a].y() - m_nodes[i].y(), 2));
        }
    }

    try {
        IloModel mod(env);

        IloFloatVar z(env);
        IloArray<IloBoolVarArray> x_agent_node(env);
        for (int a = 0; a < m_agents.size(); a++) {
            x_agent_node.add(IloBoolVarArray(env, m_nodes.size()));
        }

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

        env.out() << "Minimume Cost = " << cplex.getObjValue() << endl;

        for (int a = 0; a < m_agents.size(); a++) {
            for (int i = 0; i < m_nodes.size(); i++) {
                if (cplex.getValue(x_agent_node[a][i])) {
                    m_agent_paths[a] << QPair<quint32, quint32>(i, i);
                }
            }
        }
    }
    catch (IloException& e) {
        cerr << "Concert exception caught: " << e << endl;
    }
    catch (...) {
        cerr << "Unknown exception caught" << endl;
    }

    env.end();

    return result;
}

bool UBPlanner::planAgent(quint32 agent) {
    bool result = false;

    IloEnv env;

    qreal dist = 0;
    qreal min_dist = sqrt(pow(m_agents[agent].x() - m_nodes[m_agent_paths[agent][0].first].x(), 2) + pow(m_agents[agent].y() - m_nodes[m_agent_paths[agent][0].first].y(), 2));
    m_depots[agent] = m_agent_paths[agent][0].first;
    QPair<quint32, quint32> node;
    foreach (node, m_agent_paths[agent]) {
        dist = sqrt(pow(m_agents[agent].x() - m_nodes[node.first].x(), 2) + pow(m_agents[agent].y() - m_nodes[node.first].y(), 2));

        if (dist < min_dist) {
            min_dist = dist;
            m_depots[agent] = node.first;
        }
    }

    IloIntArray2 dist_node_node(env);
    for (int i = 0; i < m_agent_paths[agent].size(); i++) {
        dist_node_node.add(IloIntArray(env, m_agent_paths[agent].size()));
    }

    IloIntArray3 direct_node_node_node(env);
    for (int i = 0; i < m_agent_paths[agent].size(); i++) {
        IloIntArray2 direct_node_node(env);
        for (int j = 0; j < m_agent_paths[agent].size(); j++) {
            direct_node_node.add(IloIntArray(env, m_agent_paths[agent].size()));
        }

        direct_node_node_node.add(direct_node_node);
    }

    qreal max_dist = (1.0 + sqrt(2.0) / 2.0) * m_dim;

    for (int i = 0; i < m_agent_paths[agent].size(); i++) {
        for (int j = 0; j < m_agent_paths[agent].size(); j++) {
            qreal dist = sqrt(pow(m_nodes[m_agent_paths[agent][i].first].x() - m_nodes[m_agent_paths[agent][j].first].x(), 2) + pow(m_nodes[m_agent_paths[agent][i].first].y() - m_nodes[m_agent_paths[agent][j].first].y(), 2));

            if (!dist || dist > max_dist) {
                dist_node_node[i][j] = m_kappa;
            } else {
                dist_node_node[i][j] = m_pcs * dist;
            }
        }
    }

    for (int i = 0; i < m_agent_paths[agent].size(); i++) {
        for (int j = 0; j < m_agent_paths[agent].size(); j++) {
            for (int k = 0; k < m_agent_paths[agent].size(); k++) {
                if (dist_node_node[i][j] == m_kappa || dist_node_node[j][k] == m_kappa) {
                    direct_node_node_node[i][j][k] = 0;
                } else {
                    qreal r = pow(m_nodes[m_agent_paths[agent][i].first].x() - m_nodes[m_agent_paths[agent][j].first].x(), 2) + pow(m_nodes[m_agent_paths[agent][i].first].y() - m_nodes[m_agent_paths[agent][j].first].y(), 2);
                    qreal s = pow(m_nodes[m_agent_paths[agent][j].first].x() - m_nodes[m_agent_paths[agent][k].first].x(), 2) + pow(m_nodes[m_agent_paths[agent][j].first].y() - m_nodes[m_agent_paths[agent][k].first].y(), 2);
                    qreal t = pow(m_nodes[m_agent_paths[agent][k].first].x() - m_nodes[m_agent_paths[agent][i].first].x(), 2) + pow(m_nodes[m_agent_paths[agent][k].first].y() - m_nodes[m_agent_paths[agent][i].first].y(), 2);

                    direct_node_node_node[i][j][k] = m_pcs * (M_PI - acos((r + s - t) / sqrt(4.0 * r * s)));
                }
            }
        }
    }

    try {
        IloModel mod(env);

        IloNumVarArray u(env, m_agent_paths[agent].size(), 0.0, IloInfinity, ILOFLOAT);
        IloArray<IloBoolVarArray> x_node_node(env);
        for (int i = 0; i < m_agent_paths[agent].size(); i++) {
            x_node_node.add(IloBoolVarArray(env, m_agent_paths[agent].size()));
        }

        IloExpr total_dist(env), total_direct(env);

        for (int i = 0; i < m_agent_paths[agent].size(); i++) {
            for (int j = 0; j < m_agent_paths[agent].size(); j++) {
                if (j == i) {
                    continue;
                }

                total_dist += dist_node_node[i][j] * x_node_node[i][j];
            }
        }

        for (int i = 0; i < m_agent_paths[agent].size(); i++) {
            for (int j = 0; j < m_agent_paths[agent].size(); j++) {
                if (j == i || m_agent_paths[agent][j].first == m_depots[agent]) {
                    continue;
                }

                for (int k = 0; k < m_agent_paths[agent].size(); k++) {
                    if (k == j) {
                        continue;
                    }

                    total_direct += direct_node_node_node[i][j][k] * x_node_node[i][j] * x_node_node[j][k];
                }
            }
        }

        mod.add(IloMinimize(env, m_lambda * total_dist + m_gamma * total_direct));

        total_dist.end();
        total_direct.end();

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
        if (!cplex.solve() || cplex.getObjValue() / m_pcs >= m_kappa) {
            throw(-1);
        }

        result = true;

        env.out() << "Minimume Cost = " << cplex.getObjValue() / m_pcs << endl;

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
        cerr << "Concert exception caught: " << e << endl;
    }
    catch (...) {
        cerr << "Unknown exception caught" << endl;
    }

    env.end();

    return result;
}

void UBPlanner::pathInfo(quint32 agent) {
    qreal dist = 0;
    qreal direct = 0;

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

    while (true) {
        dist += sqrt(pow(m_nodes[i].x() - m_nodes[j].x(), 2) + pow(m_nodes[i].y() - m_nodes[j].y(), 2));

        if (j == m_depots[agent]) {
            break;
        }

        for (int node = 0; node < m_agent_paths[agent].size(); node++) {
            if (m_agent_paths[agent][node].first == j) {
                k = m_agent_paths[agent][node].second;

                break;
            }
        }

        qreal r = pow(m_nodes[i].x() - m_nodes[j].x(), 2) + pow(m_nodes[i].y() - m_nodes[j].y(), 2);
        qreal s = pow(m_nodes[j].x() - m_nodes[k].x(), 2) + pow(m_nodes[j].y() - m_nodes[k].y(), 2);
        qreal t = pow(m_nodes[k].x() - m_nodes[i].x(), 2) + pow(m_nodes[k].y() - m_nodes[i].y(), 2);

        qreal q = M_PI - acos((r + s - t) / sqrt(4.0 * r * s));

        direct += q;

        if (q > M_PI / 4.0 - M_PI / 8.0 && q < M_PI / 4.0 + M_PI / 8.0) {
            ang1++;
        } else if (q > M_PI / 2.0 - M_PI / 8.0 && q < M_PI / 2.0 + M_PI / 8.0) {
            ang2++;
        } else if (q > 3.0 * M_PI / 4.0 - M_PI / 8.0 && q < 3.0 * M_PI / 4.0 + M_PI / 8.0) {
            ang3++;
        }

        i = j;
        j = k;
    }

    cout << "Total Distance: " << dist << " | Number of 45' Turn: " << ang1 << " | Number of 90' Turn: " << ang2 << " | Number of 135' Turn: " << ang3 << endl;
    cout << "Total Cost: " << m_lambda * dist + m_gamma * direct << endl;
}

void UBPlanner::missionAgent(quint32 agent) {
    UASWaypointManager wpm;
    internals::PointLatLng pll;
    projections::MercatorProjection proj;

    pll = proj.FromPixelToLatLng(m_nodes[m_depots[agent]].x(), m_nodes[m_depots[agent]].y(), GND_RES);

    Waypoint* wp = wpm.createWaypoint();
//    wp.setFrame(MAV_FRAME_GLOBAL_RELATIVE_ALT);
    wp->setAcceptanceRadius(POINT_ZONE);
    wp->setLatitude(pll.Lat());
    wp->setLongitude(pll.Lng());

    wp = wpm.createWaypoint();
    wp->setAction(MAV_CMD_NAV_TAKEOFF);
    wp->setAcceptanceRadius(POINT_ZONE);
    wp->setLatitude(pll.Lat());
    wp->setLongitude(pll.Lng());
    wp->setAltitude(TAKEOFF_ALT);

    quint32 node = m_depots[agent];
    while (true) {
        for (int i = 0; i < m_agent_paths[agent].size(); i++) {
            if (m_agent_paths[agent][i].first == node) {
                node = m_agent_paths[agent][i].second;

                break;
            }
        }

        pll = proj.FromPixelToLatLng(m_nodes[node].x(), m_nodes[node].y(), GND_RES);

        wp = wpm.createWaypoint();
        wp->setAction(MAV_CMD_NAV_WAYPOINT);
        wp->setAcceptanceRadius(POINT_ZONE);
        wp->setLatitude(pll.Lat());
        wp->setLongitude(pll.Lng());
        wp->setAltitude(TAKEOFF_ALT);

        if (node == m_depots[agent]) {
            break;
        }
    }

    wp = wpm.createWaypoint();
    wp->setAction(MAV_CMD_NAV_LAND);
    wp->setAcceptanceRadius(POINT_ZONE);
    wp->setLatitude(pll.Lat());
    wp->setLongitude(pll.Lng());

    wpm.saveWaypoints(tr("mission_%1.txt").arg(agent));

    while (wpm.getWaypointEditableList().count()) {
        wpm.removeWaypoint(0);
    }
}

void UBPlanner::plan() {
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
        if (!planAgent(a)) {
            cerr << "Unable to plan the coverage path for agent " << a << " !" <<endl;
            exit(EXIT_FAILURE);
        }

        cout << "Elapsed time for agent " << a << " is "<< agent_time.elapsed() / 1000.0 << endl;

        pathInfo(a);
        missionAgent(a);
    }

    emit planReady();

    cout << "The planner has successfully planned the mission for each agent in total time " << total_time.elapsed() / 1000.0 << endl;
    exit(EXIT_SUCCESS);
}
