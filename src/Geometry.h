#pragma once

namespace Geometry{
    constexpr u_int_16_t N_CELLS[2] = {1000, 1000};
    constexpr int CELL_SIZE =  7;
    constexpr int BOX[2] = {CELL_SIZE*N_CELLS[0], CELL_SIZE*N_CELLS[1]};
}; //! Namespace Geometry

