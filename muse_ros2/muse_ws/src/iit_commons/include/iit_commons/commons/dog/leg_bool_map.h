#ifndef _IIT_COMMONS_DOG_LEGBOOLMAP_H_
#define _IIT_COMMONS_DOG_LEGBOOLMAP_H_

#include <iit/commons/dog/leg_data_map.h>

namespace iit {

namespace dog {

/**
 * @brief The LegBoolMap class specializes the LegDataMap with extended
 * functionalities for logic operations when the type is a bool, including:
 * negation, or, and, cwise or and cwise and.
 */
class LegBoolMap : public LegDataMap<bool> {

public:
    LegBoolMap() : LegDataMap<bool>(false) {}
    LegBoolMap(const bool& defaultValue) : LegDataMap<bool>(defaultValue) {}
    LegBoolMap(const LegBoolMap& rhs) : LegDataMap<bool>(rhs) {}

    inline LegBoolMap operator!() const {
        LegBoolMap res;
        res[LF] = !(*this)[LF];
        res[RF] = !(*this)[RF];
        res[LH] = !(*this)[LH];
        res[RH] = !(*this)[RH];
        return res;
    }

    inline LegBoolMap operator || (const LegBoolMap& rhs) const {
        LegBoolMap res;
        res[LF] = (*this)[LF] || rhs[LF];
        res[RF] = (*this)[RF] || rhs[RF];
        res[LH] = (*this)[LH] || rhs[LH];
        res[RH] = (*this)[RH] || rhs[RH];
        return res;
    }

    inline LegBoolMap operator && (const LegBoolMap& rhs) const {
        LegBoolMap res;
        res[LF] = (*this)[LF] && rhs[LF];
        res[RF] = (*this)[RF] && rhs[RF];
        res[LH] = (*this)[LH] && rhs[LH];
        res[RH] = (*this)[RH] && rhs[RH];
        return res;
    }

    static inline bool OR(const LegDataMap<bool>& rhs) {
        return(rhs[LF] || rhs[RF] || rhs[LH] || rhs[RH]);
    }

    inline bool OR() const {
        return ((*this)[LF] || (*this)[RF] || (*this)[LH] || (*this)[RH]);
    }


    static inline bool AND(const LegDataMap<bool>& rhs) {
        return(rhs[LF] && rhs[RF] && rhs[LH] && rhs[RH]);
    }

    inline bool AND() const {
        return ((*this)[LF] && (*this)[RF] && (*this)[LH] && (*this)[RH]);
    }
};

}

}

#endif
