/*
 * File: policy_in.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_POLICY_IN_H
#define HINACLOTH_POLICY_IN_H

namespace sim
{
    enum class DataLayout { Auto, SoA, AoS, Blocked };

    enum class Backend { Auto, Native, AVX2, TBB, GPU };

    enum class TimeStepper { Auto, Symplectic, SemiImplicit, Explicit };

    struct PolicyExec
    {
        DataLayout layout;
        Backend backend;
        int threads;
        bool deterministic;
        bool telemetry;
    };

    struct PolicySolve
    {
        int substeps;
        int iterations;
        float damping;
        TimeStepper stepper;
    };

    struct Policy
    {
        PolicyExec exec;
        PolicySolve solve;
    };
}

#endif
