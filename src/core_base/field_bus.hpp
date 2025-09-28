#ifndef RPHYS_FIELD_BUS_HPP
#define RPHYS_FIELD_BUS_HPP

#include <cstddef>
#include <string_view>

namespace rphys {

struct field_bus_entry {
    const char* name{};
    const void* data{};
    std::size_t count{};
    std::size_t stride{};
};

struct field_bus;

// Registration / lookup interfaces (no implementation in skeleton)
void register_field(field_bus*, const field_bus_entry&);
const field_bus_entry* find_field(const field_bus*, std::string_view name);

} // namespace rphys

#endif // RPHYS_FIELD_BUS_HPP

