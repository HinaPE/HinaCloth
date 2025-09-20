#ifndef HINACLOTH_STATUS_H
#define HINACLOTH_STATUS_H

namespace sim {
    enum class Status {
        Ok,
        InvalidArgs,
        ValidationFailed,
        NoBackend,
        Unsupported,
        OOM,
        NotReady,
        Busy
    };

    template <class T>
    struct [[nodiscard]] Result {
        Status status;
        T value;
    };
}
#endif //HINACLOTH_STATUS_H
