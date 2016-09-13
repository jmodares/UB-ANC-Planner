DEFINES += IL_STD

CPXROOT = /home/jalil/ibm/ILOG/CPLEX_Studio1263

LIBS += -lconcert -lilocplex -lcplex \
    -L$${CPXROOT}/concert/lib/x86-64_linux/static_pic \
    -L$${CPXROOT}/cplex/lib/x86-64_linux/static_pic \

INCLUDEPATH += $${CPXROOT}/concert/include \
    $${CPXROOT}/cplex/include \

DEPENDPATH += $${CPXROOT}/concert/include \
    $${CPXROOT}/cplex/include \
