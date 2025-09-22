/*
 * File: space_in.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_SPACE_IN_H
#define HINACLOTH_SPACE_IN_H

namespace sim
{
    enum class SpaceType { Auto, Lagrangian, Eulerian, Spectral };

    struct SpaceDesc
    {
        SpaceType type;
        int order;
        int refinement_level;
    };
}

#endif
