#ifndef RPHYS_API_STATUS_H
#define RPHYS_API_STATUS_H

namespace rphys {

enum class status_code {
    ok = 0,
    invalid_args,
    not_found,
    failed
};

} // namespace rphys

#endif // RPHYS_API_STATUS_H

