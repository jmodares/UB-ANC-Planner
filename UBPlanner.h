#ifndef UBPLANNER_H
#define UBPLANNER_H

#include <QPair>
#include <QVector>
#include <QPolygonF>
#include <QGeoCoordinate>

#include <QObject>

class Waypoint;

class UBPlanner : public QObject
{
    Q_OBJECT
public:
    explicit UBPlanner(QObject *parent = 0);

private:

signals:
    void planReady();

public slots:
    void plan();
    void startPlanner();

protected:
    QString m_file;
    quint32 m_res;
    quint32 m_limit;
    qreal m_gap;
    quint32 m_lambda;
    quint32 m_gamma;
    quint32 m_kappa;

    quint32 m_pcs;

    QVector<quint32> m_depots;
    QVector<QPolygonF> m_areas;

    QVector<QGeoCoordinate> m_nodes;
    QVector<QGeoCoordinate> m_agents;

    QVector<QVector<QPair<quint32, quint32> > > m_agent_paths;

protected:
    bool divide();
    void decompose();
    bool pathInfo(quint32 agent);
    void missionAgent(quint32 agent);
    bool evaluate(const QVector<QPointF>& cell);

    QList<Waypoint*> loadWaypoints(const QString &loadFile);
    void storeWaypoints(const QString& storeFile, QList<Waypoint*>& wps);

    virtual bool planAgent(quint32 agent);

public:
    void setFile(const QString& file) {m_file = file;}
    void setResolution(quint32 res) {m_res = res;}
    void setLimit(quint32 limit) {m_limit = limit;}
    void setGap(qreal gap) {m_gap = gap;}
    void setLambda(quint32 lambda) {m_lambda = lambda;}
    void setGamma(quint32 gamma) {m_gamma = gamma;}
    void setkappa(quint32 kappa) {m_kappa = kappa;}
};

#endif // UBPLANNER_H
