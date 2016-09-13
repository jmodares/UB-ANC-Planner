#ifndef UBPLANNER_H
#define UBPLANNER_H

#include <QPair>
#include <QPointF>
#include <QVector>
#include <QPolygonF>

#include <QObject>

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
    quint32 m_lambda;
    quint32 m_gamma;
    quint64 m_kappa;
    quint32 m_dim;

    quint32 m_pcs;

    qreal m_gap;
    quint32 m_limit;

    QVector<quint32> m_depots;
    QVector<QPointF> m_nodes;
    QVector<QPointF> m_agents;
    QVector<QPolygonF> m_areas;

    QVector<QVector<QPair<quint32, quint32> > > m_agent_paths;

protected:
    bool divide();
    void decompose();
    void pathInfo(quint32 agent);
    bool planAgent(quint32 agent);
    void missionAgent(quint32 agent);
    bool evaluate(const QVector<QPointF>& cell);
};

#endif // UBPLANNER_H
