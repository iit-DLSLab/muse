pub mod rmw {
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[link(name = "state_estimator_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__state_estimator_msgs__msg__Attitude() -> *const std::ffi::c_void;
}

#[link(name = "state_estimator_msgs__rosidl_generator_c")]
extern "C" {
    fn state_estimator_msgs__msg__Attitude__init(msg: *mut Attitude) -> bool;
    fn state_estimator_msgs__msg__Attitude__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Attitude>, size: usize) -> bool;
    fn state_estimator_msgs__msg__Attitude__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Attitude>);
    fn state_estimator_msgs__msg__Attitude__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Attitude>, out_seq: *mut rosidl_runtime_rs::Sequence<Attitude>) -> bool;
}

// Corresponds to state_estimator_msgs__msg__Attitude
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Attitude {
    pub header: std_msgs::msg::rmw::Header,
    pub quaternion: [f64; 4],
    pub roll_deg: f64,
    pub pitch_deg: f64,
    pub yaw_deg: f64,
    pub angular_velocity: [f64; 3],
}



impl Default for Attitude {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !state_estimator_msgs__msg__Attitude__init(&mut msg as *mut _) {
        panic!("Call to state_estimator_msgs__msg__Attitude__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Attitude {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { state_estimator_msgs__msg__Attitude__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { state_estimator_msgs__msg__Attitude__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { state_estimator_msgs__msg__Attitude__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Attitude {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Attitude where Self: Sized {
  const TYPE_NAME: &'static str = "state_estimator_msgs/msg/Attitude";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__state_estimator_msgs__msg__Attitude() }
  }
}


#[link(name = "state_estimator_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__state_estimator_msgs__msg__ContactDetection() -> *const std::ffi::c_void;
}

#[link(name = "state_estimator_msgs__rosidl_generator_c")]
extern "C" {
    fn state_estimator_msgs__msg__ContactDetection__init(msg: *mut ContactDetection) -> bool;
    fn state_estimator_msgs__msg__ContactDetection__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ContactDetection>, size: usize) -> bool;
    fn state_estimator_msgs__msg__ContactDetection__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ContactDetection>);
    fn state_estimator_msgs__msg__ContactDetection__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ContactDetection>, out_seq: *mut rosidl_runtime_rs::Sequence<ContactDetection>) -> bool;
}

// Corresponds to state_estimator_msgs__msg__ContactDetection
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ContactDetection {
    pub header: std_msgs::msg::rmw::Header,
    pub stance_lf: bool,
    pub stance_rf: bool,
    pub stance_lh: bool,
    pub stance_rh: bool,
}



impl Default for ContactDetection {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !state_estimator_msgs__msg__ContactDetection__init(&mut msg as *mut _) {
        panic!("Call to state_estimator_msgs__msg__ContactDetection__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ContactDetection {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { state_estimator_msgs__msg__ContactDetection__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { state_estimator_msgs__msg__ContactDetection__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { state_estimator_msgs__msg__ContactDetection__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ContactDetection {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ContactDetection where Self: Sized {
  const TYPE_NAME: &'static str = "state_estimator_msgs/msg/ContactDetection";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__state_estimator_msgs__msg__ContactDetection() }
  }
}


#[link(name = "state_estimator_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__state_estimator_msgs__msg__JointStateWithAcceleration() -> *const std::ffi::c_void;
}

#[link(name = "state_estimator_msgs__rosidl_generator_c")]
extern "C" {
    fn state_estimator_msgs__msg__JointStateWithAcceleration__init(msg: *mut JointStateWithAcceleration) -> bool;
    fn state_estimator_msgs__msg__JointStateWithAcceleration__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<JointStateWithAcceleration>, size: usize) -> bool;
    fn state_estimator_msgs__msg__JointStateWithAcceleration__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<JointStateWithAcceleration>);
    fn state_estimator_msgs__msg__JointStateWithAcceleration__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<JointStateWithAcceleration>, out_seq: *mut rosidl_runtime_rs::Sequence<JointStateWithAcceleration>) -> bool;
}

// Corresponds to state_estimator_msgs__msg__JointStateWithAcceleration
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointStateWithAcceleration {
    pub header: std_msgs::msg::rmw::Header,
    pub name: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,
    pub position: rosidl_runtime_rs::Sequence<f64>,
    pub velocity: rosidl_runtime_rs::Sequence<f64>,
    pub acceleration: rosidl_runtime_rs::Sequence<f64>,
    pub effort: rosidl_runtime_rs::Sequence<f64>,
}



impl Default for JointStateWithAcceleration {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !state_estimator_msgs__msg__JointStateWithAcceleration__init(&mut msg as *mut _) {
        panic!("Call to state_estimator_msgs__msg__JointStateWithAcceleration__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for JointStateWithAcceleration {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { state_estimator_msgs__msg__JointStateWithAcceleration__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { state_estimator_msgs__msg__JointStateWithAcceleration__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { state_estimator_msgs__msg__JointStateWithAcceleration__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for JointStateWithAcceleration {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for JointStateWithAcceleration where Self: Sized {
  const TYPE_NAME: &'static str = "state_estimator_msgs/msg/JointStateWithAcceleration";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__state_estimator_msgs__msg__JointStateWithAcceleration() }
  }
}


#[link(name = "state_estimator_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__state_estimator_msgs__msg__LegOdometry() -> *const std::ffi::c_void;
}

#[link(name = "state_estimator_msgs__rosidl_generator_c")]
extern "C" {
    fn state_estimator_msgs__msg__LegOdometry__init(msg: *mut LegOdometry) -> bool;
    fn state_estimator_msgs__msg__LegOdometry__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LegOdometry>, size: usize) -> bool;
    fn state_estimator_msgs__msg__LegOdometry__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LegOdometry>);
    fn state_estimator_msgs__msg__LegOdometry__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LegOdometry>, out_seq: *mut rosidl_runtime_rs::Sequence<LegOdometry>) -> bool;
}

// Corresponds to state_estimator_msgs__msg__LegOdometry
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LegOdometry {
    pub header: std_msgs::msg::rmw::Header,
    pub lin_vel_lf: [f64; 3],
    pub lin_vel_rf: [f64; 3],
    pub lin_vel_lh: [f64; 3],
    pub lin_vel_rh: [f64; 3],
    pub base_velocity: [f64; 3],
}



impl Default for LegOdometry {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !state_estimator_msgs__msg__LegOdometry__init(&mut msg as *mut _) {
        panic!("Call to state_estimator_msgs__msg__LegOdometry__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LegOdometry {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { state_estimator_msgs__msg__LegOdometry__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { state_estimator_msgs__msg__LegOdometry__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { state_estimator_msgs__msg__LegOdometry__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LegOdometry {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LegOdometry where Self: Sized {
  const TYPE_NAME: &'static str = "state_estimator_msgs/msg/LegOdometry";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__state_estimator_msgs__msg__LegOdometry() }
  }
}


#[link(name = "state_estimator_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__state_estimator_msgs__msg__SensorFusion() -> *const std::ffi::c_void;
}

#[link(name = "state_estimator_msgs__rosidl_generator_c")]
extern "C" {
    fn state_estimator_msgs__msg__SensorFusion__init(msg: *mut SensorFusion) -> bool;
    fn state_estimator_msgs__msg__SensorFusion__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SensorFusion>, size: usize) -> bool;
    fn state_estimator_msgs__msg__SensorFusion__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SensorFusion>);
    fn state_estimator_msgs__msg__SensorFusion__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SensorFusion>, out_seq: *mut rosidl_runtime_rs::Sequence<SensorFusion>) -> bool;
}

// Corresponds to state_estimator_msgs__msg__SensorFusion
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SensorFusion {
    pub header: std_msgs::msg::rmw::Header,
    pub position: [f64; 3],
    pub linear_velocity: [f64; 3],
}



impl Default for SensorFusion {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !state_estimator_msgs__msg__SensorFusion__init(&mut msg as *mut _) {
        panic!("Call to state_estimator_msgs__msg__SensorFusion__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SensorFusion {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { state_estimator_msgs__msg__SensorFusion__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { state_estimator_msgs__msg__SensorFusion__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { state_estimator_msgs__msg__SensorFusion__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SensorFusion {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SensorFusion where Self: Sized {
  const TYPE_NAME: &'static str = "state_estimator_msgs/msg/SensorFusion";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__state_estimator_msgs__msg__SensorFusion() }
  }
}


}  // mod rmw


#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Attitude {
    pub header: std_msgs::msg::Header,
    pub quaternion: [f64; 4],
    pub roll_deg: f64,
    pub pitch_deg: f64,
    pub yaw_deg: f64,
    pub angular_velocity: [f64; 3],
}



impl Default for Attitude {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::Attitude::default())
  }
}

impl rosidl_runtime_rs::Message for Attitude {
  type RmwMsg = crate::msg::rmw::Attitude;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        quaternion: msg.quaternion,
        roll_deg: msg.roll_deg,
        pitch_deg: msg.pitch_deg,
        yaw_deg: msg.yaw_deg,
        angular_velocity: msg.angular_velocity,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        quaternion: msg.quaternion,
      roll_deg: msg.roll_deg,
      pitch_deg: msg.pitch_deg,
      yaw_deg: msg.yaw_deg,
        angular_velocity: msg.angular_velocity,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      quaternion: msg.quaternion,
      roll_deg: msg.roll_deg,
      pitch_deg: msg.pitch_deg,
      yaw_deg: msg.yaw_deg,
      angular_velocity: msg.angular_velocity,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ContactDetection {
    pub header: std_msgs::msg::Header,
    pub stance_lf: bool,
    pub stance_rf: bool,
    pub stance_lh: bool,
    pub stance_rh: bool,
}



impl Default for ContactDetection {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::ContactDetection::default())
  }
}

impl rosidl_runtime_rs::Message for ContactDetection {
  type RmwMsg = crate::msg::rmw::ContactDetection;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        stance_lf: msg.stance_lf,
        stance_rf: msg.stance_rf,
        stance_lh: msg.stance_lh,
        stance_rh: msg.stance_rh,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      stance_lf: msg.stance_lf,
      stance_rf: msg.stance_rf,
      stance_lh: msg.stance_lh,
      stance_rh: msg.stance_rh,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      stance_lf: msg.stance_lf,
      stance_rf: msg.stance_rf,
      stance_lh: msg.stance_lh,
      stance_rh: msg.stance_rh,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointStateWithAcceleration {
    pub header: std_msgs::msg::Header,
    pub name: Vec<std::string::String>,
    pub position: Vec<f64>,
    pub velocity: Vec<f64>,
    pub acceleration: Vec<f64>,
    pub effort: Vec<f64>,
}



impl Default for JointStateWithAcceleration {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::JointStateWithAcceleration::default())
  }
}

impl rosidl_runtime_rs::Message for JointStateWithAcceleration {
  type RmwMsg = crate::msg::rmw::JointStateWithAcceleration;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        name: msg.name
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        position: msg.position.into(),
        velocity: msg.velocity.into(),
        acceleration: msg.acceleration.into(),
        effort: msg.effort.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        name: msg.name
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        position: msg.position.as_slice().into(),
        velocity: msg.velocity.as_slice().into(),
        acceleration: msg.acceleration.as_slice().into(),
        effort: msg.effort.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      name: msg.name
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
      position: msg.position
          .into_iter()
          .collect(),
      velocity: msg.velocity
          .into_iter()
          .collect(),
      acceleration: msg.acceleration
          .into_iter()
          .collect(),
      effort: msg.effort
          .into_iter()
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LegOdometry {
    pub header: std_msgs::msg::Header,
    pub lin_vel_lf: [f64; 3],
    pub lin_vel_rf: [f64; 3],
    pub lin_vel_lh: [f64; 3],
    pub lin_vel_rh: [f64; 3],
    pub base_velocity: [f64; 3],
}



impl Default for LegOdometry {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::LegOdometry::default())
  }
}

impl rosidl_runtime_rs::Message for LegOdometry {
  type RmwMsg = crate::msg::rmw::LegOdometry;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        lin_vel_lf: msg.lin_vel_lf,
        lin_vel_rf: msg.lin_vel_rf,
        lin_vel_lh: msg.lin_vel_lh,
        lin_vel_rh: msg.lin_vel_rh,
        base_velocity: msg.base_velocity,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        lin_vel_lf: msg.lin_vel_lf,
        lin_vel_rf: msg.lin_vel_rf,
        lin_vel_lh: msg.lin_vel_lh,
        lin_vel_rh: msg.lin_vel_rh,
        base_velocity: msg.base_velocity,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      lin_vel_lf: msg.lin_vel_lf,
      lin_vel_rf: msg.lin_vel_rf,
      lin_vel_lh: msg.lin_vel_lh,
      lin_vel_rh: msg.lin_vel_rh,
      base_velocity: msg.base_velocity,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SensorFusion {
    pub header: std_msgs::msg::Header,
    pub position: [f64; 3],
    pub linear_velocity: [f64; 3],
}



impl Default for SensorFusion {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::SensorFusion::default())
  }
}

impl rosidl_runtime_rs::Message for SensorFusion {
  type RmwMsg = crate::msg::rmw::SensorFusion;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        position: msg.position,
        linear_velocity: msg.linear_velocity,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        position: msg.position,
        linear_velocity: msg.linear_velocity,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      position: msg.position,
      linear_velocity: msg.linear_velocity,
    }
  }
}


