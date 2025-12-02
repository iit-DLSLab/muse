#ifndef DOG_HPP_VBIGSIM4
#define DOG_HPP_VBIGSIM4

// =============================================================================
// Includes
// =============================================================================
#include <memory>

#include "iit/commons/dog/body_lengths.h"
// #include "iit/commons/dog/declarations.h"
#include "iit/commons/dog/feet_contact_forces.h"
#include "iit/commons/dog/feet_jacobians.h"
#include "iit/commons/dog/forward_kinematics.h"
#include "iit/commons/dog/inertia_properties.h"
#include "iit/commons/dog/inverse_dynamics.h"
#include "iit/commons/dog/inverse_kinematics.h"
#include "iit/commons/dog/joint_bool_map.h"
// #include "iit/commons/dog/joint_data_map.h"
// #include "iit/commons/dog/joint_id_tricks.h"
#include "iit/commons/dog/jsim.h"
#include "iit/commons/dog/kin_dyn_params.h"
#include "iit/commons/dog/leg_bool_map.h"
// #include "iit/commons/dog/leg_data_map.h"
// #include "iit/commons/dog/link_data_map.h"
#include "iit/commons/dog/robot_limits.h"
#include "iit/commons/dog/shin_jacobians.h"
#include "iit/commons/dog/test_fk.h"
#include "iit/commons/dog/test_k.h"
#include "iit/commons/dog/transforms.h"

// =============================================================================
// Dog Class
// =============================================================================
// Note for the lost souls who enter here:
// These classes should not be classes inside Dog, but their member functions
// should be member functions within dog.
// However, due to the state of the code, the current implementation was forced.
// A later refactor should address this issue

/// An abstract base class for a four-legged robot
///
/// Well, the class is not abstract, but it should be. At least every class
/// inside this class is abstract, so, I guess there's that at least.
namespace iit
{
namespace dog
{

class Dog
{
public:
	// ============================ Constructors ===============================
	Dog
	(
		std::shared_ptr<JSimBase>                   pJSimBase_
		std::shared_ptr<TestFK>                     pTestFK_
		std::shared_ptr<JointBoolMap>               pJointBoolMap_
		std::shared_ptr<RobotLengths>               pRobotLengths_
		std::shared_ptr<ForwardKinematics>          pForwardKinematics_
		std::shared_ptr<TestK>                      pTestK_
		std::shared_ptr<FeetContactForces>          pFeetContactForces_
		std::shared_ptr<InverseKinematics>          pInverseKinematics_
		std::shared_ptr<InverseDynamicsBase>        pInverseDynamicsBase_
		std::shared_ptr<HomogeneousTransformsBase>  pHomogeneousTransformsBase_
		std::shared_ptr<MotionTransformsBase>       pMotionTransformsBase_
		std::shared_ptr<ForceTransformsBase>        pForceTransformsBase_
		std::shared_ptr<KinDynParams>               pKinDynParams_
		std::shared_ptr<LimitsBase>                 pLimitsBase_
		// std::shared_ptr<LegDataMap>                 pLegDataMap_
		std::shared_ptr<InertiaPropertiesBase>     pInertiaPropertiesBase_
		std::shared_ptr<FeetJacobians>              pFeetJacobians_
		std::shared_ptr<LegBoolMap>                 pLegBoolMap_
		// std::shared_ptr<LinkDataMap>                pLinkDataMap_
		std::shared_ptr<ShinJacobians>              pShinJacobians_
	) :
		pJSimBase(pJSimBase_),
		pTestFK(pTestFK_),
		pJointBoolMap(pJointBoolMap_),
		pRobotLengths(pRobotLengths_),
		pForwardKinematics(pForwardKinematics_),
		pTestK(pTestK_),
		pFeetContactForces(pFeetContactForces_),
		pInverseKinematics(pInverseKinematics_),
		pInverseDynamicsBase(pInverseDynamicsBase_),
		pHomogeneousTransformsBase(pHomogeneousTransformsBase_),
		pMotionTransformsBase(pMotionTransformsBase_),
		pForceTransformsBase(pForceTransformsBase_),
		pKinDynParams(pKinDynParams_),
		pLimitsBase(pLimitsBase_),
		// pLegDataMap(pLegDataMap_),
		pInertiaPropertiesBase(pInertiaPropertiesBase_),
		pFeetJacobians(pFeetJacobians_),
		pLegBoolMap(pLegBoolMap_),
		// pLinkDataMap(pLinkDataMap_),
		pShinJacobians(pShinJacobians_)

	{ }

	virtual ~Dog();

	// ============================= Getters ===================================
	std::shared_ptr<JSimBase>                   getJSimBase()                   {  return  this->pJSimBase;                   }
	std::shared_ptr<TestFK>                     getTestFK()                     {  return  this->pTestFK;                     }
	std::shared_ptr<JointBoolMap>               getJointBoolMap()               {  return  this->pJointBoolMap;               }
	std::shared_ptr<RobotLengths>               getRobotLengths()               {  return  this->pRobotLengths;               }
	std::shared_ptr<ForwardKinematics>          getForwardKinematics()          {  return  this->pForwardKinematics;          }
	std::shared_ptr<TestK>                      getTestK()                      {  return  this->pTestK;                      }
	std::shared_ptr<FeetContactForces>           getFeetContactForces()           {  return  this->pFeetContactForces;           }
	std::shared_ptr<InverseKinematics>          getInverseKinematics()          {  return  this->pInverseKinematics;          }
	std::shared_ptr<InverseDynamicsBase>        getInverseDynamicsBase()        {  return  this->pInverseDynamicsBase;        }
	std::shared_ptr<HomogeneousTransformsBase>  getHomogeneousTransformsBase()  {  return  this->pHomogeneousTransformsBase;  }
	std::shared_ptr<MotionTransformsBase>       getMotionTransformsBase()       {  return  this->pMotionTransformsBase;       }
	std::shared_ptr<ForceTransformsBase>        getForceTransformsBase()        {  return  this->pForceTransformsBase;        }
	std::shared_ptr<KinDynParams>               getKinDynParams()               {  return  this->pKinDynParams;               }
	std::shared_ptr<LimitsBase>                 getLimitsBase()                 {  return  this->pLimitsBase;                 }
	// std::shared_ptr<LegDataMap>                 getLegDataMap()                 {  return  this->pLegDataMap;                 }
	std::shared_ptr<InertiaPropertiesBase>     getInertiaPropertiesBase()     {  return  this->pInertiaPropertiesBase;     }
	std::shared_ptr<FeetJacobians>              getFeetJacobians()              {  return  this->pFeetJacobians;              }
	std::shared_ptr<LegBoolMap>                 getLegBoolMap()                 {  return  this->pLegBoolMap;                 }
	// std::shared_ptr<LinkDataMap>                getLinkDataMap()                {  return  this->pLinkDataMap;                }
	std::shared_ptr<ShinJacobians>              getShinJacobians()              {  return  this->pShinJacobians;              }

private:
	// ====================== Pointers to components ===========================
	const std::shared_ptr<JSimBase>                   pJSimBase;
	const std::shared_ptr<TestFK>                     pTestFK;
	const std::shared_ptr<JointBoolMap>               pJointBoolMap;
	const std::shared_ptr<RobotLengths>               pRobotLengths;
	const std::shared_ptr<ForwardKinematics>          pForwardKinematics;
	const std::shared_ptr<TestK>                      pTestK;
	const std::shared_ptr<FeetContactForces>           pFeetContactForces;
	const std::shared_ptr<InverseKinematics>          pInverseKinematics;
	const std::shared_ptr<InverseDynamicsBase>        pInverseDynamicsBase;
	const std::shared_ptr<HomogeneousTransformsBase>  pHomogeneousTransformsBase;
	const std::shared_ptr<MotionTransformsBase>       pMotionTransformsBase;
	const std::shared_ptr<ForceTransformsBase>        pForceTransformsBase;
	const std::shared_ptr<KinDynParams>               pKinDynParams;
	const std::shared_ptr<LimitsBase>                 pLimitsBase;
	// const std::shared_ptr<LegDataMap>                 pLegDataMap;
	const std::shared_ptr<InertiaPropertiesBase>     pInertiaPropertiesBase;
	const std::shared_ptr<FeetJacobians>              pFeetJacobians;
	const std::shared_ptr<LegBoolMap>                 pLegBoolMap;
	// const std::shared_ptr<LinkDataMap>                pLinkDataMap;
	const std::shared_ptr<ShinJacobians>              pShinJacobians;

};

} // namespace dog

} // namespace iit
