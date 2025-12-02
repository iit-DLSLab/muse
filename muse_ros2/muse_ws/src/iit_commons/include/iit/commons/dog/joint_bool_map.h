#ifndef _IIT_COMMONS_DOG_JOINTBOOLMAP_H_
#define _IIT_COMMONS_DOG_JOINTBOOLMAP_H_

#include <iit/commons/dog/joint_data_map.h>

namespace iit {

namespace dog {

/**
 * @brief The JointBoolMap class specializes the JointDataMap with extended
 * functionalities for logic operations when the type is a bool, including:
 * negation, or, and, cwise or and cwise and.
 */
class JointBoolMap : public JointDataMap<bool> {

public:
    JointBoolMap() : JointDataMap<bool>(false) {}
    JointBoolMap(const bool& defaultValue) : JointDataMap<bool>(defaultValue) {}
    JointBoolMap(const JointBoolMap& rhs) : JointDataMap<bool>(rhs) {}

    inline JointBoolMap operator!() const {
        JointBoolMap res;
        res[LF_HAA] = !(*this)[LF_HAA];
        res[LF_HFE] = !(*this)[LF_HFE];
        res[LF_KFE] = !(*this)[LF_KFE];
        res[RF_HAA] = !(*this)[RF_HAA];
        res[RF_HFE] = !(*this)[RF_HFE];
        res[RF_KFE] = !(*this)[RF_KFE];
        res[LH_HAA] = !(*this)[LH_HAA];
        res[LH_HFE] = !(*this)[LH_HFE];
        res[LH_KFE] = !(*this)[LH_KFE];
        res[RH_HAA] = !(*this)[RH_HAA];
        res[RH_HFE] = !(*this)[RH_HFE];
        res[RH_KFE] = !(*this)[RH_KFE];
        return res;
    }

    inline JointBoolMap operator || (const JointBoolMap& rhs) const {
        JointBoolMap res;
        res[LF_HAA] = (*this)[LF_HAA] || rhs[LF_HAA];
        res[LF_HFE] = (*this)[LF_HFE] || rhs[LF_HFE];
        res[LF_KFE] = (*this)[LF_KFE] || rhs[LF_KFE];
        res[RF_HAA] = (*this)[RF_HAA] || rhs[RF_HAA];
        res[RF_HFE] = (*this)[RF_HFE] || rhs[RF_HFE];
        res[RF_KFE] = (*this)[RF_KFE] || rhs[RF_KFE];
        res[LH_HAA] = (*this)[LH_HAA] || rhs[LH_HAA];
        res[LH_HFE] = (*this)[LH_HFE] || rhs[LH_HFE];
        res[LH_KFE] = (*this)[LH_KFE] || rhs[LH_KFE];
        res[RH_HAA] = (*this)[RH_HAA] || rhs[RH_HAA];
        res[RH_HFE] = (*this)[RH_HFE] || rhs[RH_HFE];
        res[RH_KFE] = (*this)[RH_KFE] || rhs[RH_KFE];
        return res;
    }

    inline JointBoolMap operator && (const JointBoolMap& rhs) const {
        JointBoolMap res;
        res[LF_HAA] = (*this)[LF_HAA] && rhs[LF_HAA];
        res[LF_HFE] = (*this)[LF_HFE] && rhs[LF_HFE];
        res[LF_KFE] = (*this)[LF_KFE] && rhs[LF_KFE];
        res[RF_HAA] = (*this)[RF_HAA] && rhs[RF_HAA];
        res[RF_HFE] = (*this)[RF_HFE] && rhs[RF_HFE];
        res[RF_KFE] = (*this)[RF_KFE] && rhs[RF_KFE];
        res[LH_HAA] = (*this)[LH_HAA] && rhs[LH_HAA];
        res[LH_HFE] = (*this)[LH_HFE] && rhs[LH_HFE];
        res[LH_KFE] = (*this)[LH_KFE] && rhs[LH_KFE];
        res[RH_HAA] = (*this)[RH_HAA] && rhs[RH_HAA];
        res[RH_HFE] = (*this)[RH_HFE] && rhs[RH_HFE];
        res[RH_KFE] = (*this)[RH_KFE] && rhs[RH_KFE];
        return res;
    }

    static inline bool OR(const JointDataMap<bool>& rhs) {
        return(rhs[LF_HAA] || rhs[LF_HFE] || rhs[LF_KFE] ||
               rhs[RF_HAA] || rhs[RF_HFE] || rhs[RF_KFE] ||
               rhs[LH_HAA] || rhs[LH_HFE] || rhs[LH_KFE] ||
               rhs[RH_HAA] || rhs[RH_HFE] || rhs[RH_KFE]);
    }

    inline bool OR() const {
        return ((*this)[LF_HAA] || (*this)[LF_HFE] || (*this)[LF_KFE] ||
                (*this)[RF_HAA] || (*this)[RF_HFE] || (*this)[RF_KFE] ||
                (*this)[LH_HAA] || (*this)[LH_HFE] || (*this)[LH_KFE] ||
                (*this)[RH_HAA] || (*this)[RH_HFE] || (*this)[RH_KFE]);
    }


    static inline bool AND(const JointDataMap<bool>& rhs) {
        return(rhs[LF_HAA] && rhs[LF_HFE] && rhs[LF_KFE] &&
               rhs[RF_HAA] && rhs[RF_HFE] && rhs[RF_KFE] &&
               rhs[LH_HAA] && rhs[LH_HFE] && rhs[LH_KFE] &&
               rhs[RH_HAA] && rhs[RH_HFE] && rhs[RH_KFE]);
    }

    inline bool AND() const {
        return ((*this)[LF_HAA] && (*this)[LF_HFE] && (*this)[LF_KFE] &&
                (*this)[RF_HAA] && (*this)[RF_HFE] && (*this)[RF_KFE] &&
                (*this)[LH_HAA] && (*this)[LH_HFE] && (*this)[LH_KFE] &&
                (*this)[RH_HAA] && (*this)[RH_HFE] && (*this)[RH_KFE]);
    }
};

}

}

#endif
