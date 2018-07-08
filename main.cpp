#include <QDebug>
#include <QCoreApplication>
#include <QCommandLineParser>

#include "UBPlanner.h"

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);
    QCoreApplication::setApplicationName("UB-ANC Planner");
    QCoreApplication::setApplicationVersion("1.0");

    QCommandLineParser parser;
    parser.setApplicationDescription("UB-ANC Planner CPLEX");
    parser.addHelpOption();
    parser.addVersionOption();

    parser.addOptions({
        {{"f", "file"}, "Set mission file containing area information.", "file"},
        {{"r", "resolution"}, "Set resolution of the decomposition in meters.", "resolution", "10"},
        {{"l", "limit"}, "Set optimizer time limit in seconds.", "limit", "1000000000"},
        {{"g", "gap"}, "Set gap to the optimal solution.", "gap", "0.01"},
        {{"a", "lambda"}, "Set distance factor in cost function.", "lambda", "1"},
        {{"m", "gamma"}, "Set turn factor in cost function.", "gamma", "1"},
        {{"k", "kappa"}, "Set maximum capacity for each drone.", "kappa", "1000000000"},
        {{"p", "precision"}, "Set precision for capacity calculation.", "precision", "100"},
    });

    parser.process(a);

    if (!parser.isSet("file")) {
        qWarning() << QObject::tr("The area should be set!");
        return 0;
    }

    UBPlanner* planner = new UBPlanner();
    planner->setFile(parser.value("file"));
    planner->setResolution(parser.value("resolution").toUInt());
    planner->setLimit(parser.value("limit").toUInt());
    planner->setGap(parser.value("gap").toDouble());
    planner->setLambda(parser.value("lambda").toUInt());
    planner->setGamma(parser.value("gamma").toUInt());
    planner->setkappa(parser.value("kappa").toUInt());
    planner->setPrecision(parser.value("precision").toUInt());

    planner->startPlanner();

    a.exec();
}
