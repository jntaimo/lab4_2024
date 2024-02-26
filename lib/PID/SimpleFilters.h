#ifndef SIMPLEFILTERS_H
#define SIMPLEFILTERS_H

class LeadFilter {
public:
    LeadFilter(double alpha, double Td);
    void setParameters(double alpha, double Td);
    double calculate(double input);

private:
    double _alpha;
    double _Td;
    double _lastInput;
    double _lastOutput;
    unsigned long _lastTime;
};

class LagFilter {
public:
    LagFilter(double Ti);
    void setParameters(double Ti);
    double calculate(double input);

private:
    double _Ti;
    double _lastOutput;
    unsigned long _lastTime;
};

#endif // SIMPLEFILTERS_H
