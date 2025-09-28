#ifndef RPHYS_FORWARD_H
#define RPHYS_FORWARD_H

#include <cstdint>
#include <cstddef>

namespace rphys {

struct world_id { std::uint32_t value{0}; };
struct domain_id { std::uint32_t value{0}; };
struct algorithm_id { std::uint32_t value{0}; };
struct coupling_id { std::uint32_t value{0}; };
struct field_id { std::uint32_t value{0}; };
struct param_id { std::uint32_t value{0}; };

struct world_desc { int reserved{}; }; // placeholder
struct domain_desc { int reserved{}; };
struct algorithm_desc { int reserved{}; };
struct coupling_desc { int reserved{}; };
struct field_view { const void* data{nullptr}; std::size_t count{0}; std::size_t stride{0}; };

} // namespace rphys

#endif // RPHYS_FORWARD_H
