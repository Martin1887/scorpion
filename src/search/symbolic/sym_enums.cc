#include "sym_enums.h"
#include "../plugins/plugin.h"
#include "../utils/system.h"

using namespace std;

namespace symbolic {
ostream &operator <<(ostream &os, const Dir &dir) {
    switch (dir) {
    case Dir::FW:
        return os << "fw";
    case Dir::BW:
        return os << "bw";
    case Dir::BIDIR:
        return os << "bd";
    default:
        cerr << "Name of Dir not known";
        utils::exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);
    }
}

ostream &operator <<(ostream &os, const MutexType &m) {
    switch (m) {
    case MutexType::MUTEX_NOT:
        return os << "not";
    case MutexType::MUTEX_EDELETION:
        return os << "edeletion";
    case MutexType::MUTEX_AND:
        return os << "and";
    /*case MutexType::MUTEX_RESTRICT:
        return os << "restrict";
    case MutexType::MUTEX_NPAND:
        return os << "npand";
    case MutexType::MUTEX_CONSTRAIN:
        return os << "constrain";
    case MutexType::MUTEX_LICOMP:
        return os << "licompaction";*/
    default:
        cerr << "Name of MutexType not known";
        utils::exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);
    }
}

const vector < string > MutexTypeValues {
    "MUTEX_NOT", "MUTEX_AND", "MUTEX_EDELETION",
    /*"MUTEX_RESTRICT", "MUTEX_NPAND", "MUTEX_CONSTRAIN", "MUTEX_LICOMP"*/};

const vector < string > DirValues {
    "FW", "BW", "BIDIR"
};
} // namespace symbolic

static plugins::TypedEnumPlugin<symbolic::MutexType> _enum_plugin({
    {"MUTEX_NOT", "mutex not"},
    {"MUTEX_AND", "mutex and"},
    {"MUTEX_EDELETION", "mutex edeletion"},
});
