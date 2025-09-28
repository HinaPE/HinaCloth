#ifndef RPHYS_PARAM_STORE_HPP
#define RPHYS_PARAM_STORE_HPP

#include <cstdint>
#include <string_view>

namespace rphys {

struct param_store;

// Skeleton API declarations
void ps_set_double(param_store*, std::string_view key, double value);
bool ps_get_double(const param_store*, std::string_view key, double& out_value);

} // namespace rphys

#endif // RPHYS_PARAM_STORE_HPP

