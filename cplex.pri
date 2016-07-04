DEFINES += IL_STD

LIBS += -lconcert -lilocplex -lcplex \
    -L/home/jalil/ibm/ILOG/CPLEX_Studio1263/concert/lib/x86-64_linux/static_pic \
    -L/home/jalil/ibm/ILOG/CPLEX_Studio1263/cplex/lib/x86-64_linux/static_pic \

INCLUDEPATH += /home/jalil/ibm/ILOG/CPLEX_Studio1263/concert/include \
    /home/jalil/ibm/ILOG/CPLEX_Studio1263/cplex/include \

DEPENDPATH += /home/jalil/ibm/ILOG/CPLEX_Studio1263/concert/include \
    /home/jalil/ibm/ILOG/CPLEX_Studio1263/cplex/include \
