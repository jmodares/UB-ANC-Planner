#include <QDebug>
#include <QCoreApplication>
#include <QCommandLineParser>

#include "UBPlanner.h"

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);
    QCoreApplication::setApplicationName("UB-ANC Planner");
    QCoreApplication::setApplicationVersion("1.0");

    QCommandLineParser parser;
    parser.setApplicationDescription("UB-ANC Planner LKH");
    parser.addHelpOption();
    parser.addVersionOption();

    parser.addOptions({
        {{"A", "area"}, "Set mission file containing area information", "area"},
        {{"D", "dimension"}, "Set dimension of the decomposition in meters", "dimension", "10"},
        {{"L", "limit"}, "Set optimizer time limit in seconds", "limit", "1000000000"},
        {{"P", "gap"}, "Set gap to the optimal solution", "gap", "0.01"},
        {{"M", "lambda"}, "Set distance factor in cost function", "lambda", "1"},
        {{"G", "gamma"}, "Set direction factor in cost function", "gamma", "1"},
        {{"K", "kappa"}, "Set maximum capacity for each drone", "kappa", "1000000000"},
    });

    parser.process(a);

    if (!parser.isSet("area")) {
        qWarning() << QObject::tr("The area should be set!");
        return 0;
    }

    UBPlanner* planner = new UBPlanner();
    planner->setFile(parser.value("area"));
    planner->setDimension(parser.value("dimension").toUInt());
    planner->setLimit(parser.value("limit").toUInt());
    planner->setGap(parser.value("gap").toDouble());
    planner->setLambda(parser.value("lambda").toUInt());
    planner->setGamma(parser.value("gamma").toUInt());
    planner->setkappa(parser.value("kappa").toUInt());

    planner->startPlanner();

    a.exec();
}
