// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from state_estimator_msgs:msg/ContactDetection.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "state_estimator_msgs/msg/detail/contact_detection__struct.h"
#include "state_estimator_msgs/msg/detail/contact_detection__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool state_estimator_msgs__msg__contact_detection__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
    if (class_attr == NULL) {
      return false;
    }
    PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
    if (name_attr == NULL) {
      Py_DECREF(class_attr);
      return false;
    }
    PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
    if (module_attr == NULL) {
      Py_DECREF(name_attr);
      Py_DECREF(class_attr);
      return false;
    }

    // PyUnicode_1BYTE_DATA is just a cast
    assert(strncmp("state_estimator_msgs.msg._contact_detection", (char *)PyUnicode_1BYTE_DATA(module_attr), 43) == 0);
    assert(strncmp("ContactDetection", (char *)PyUnicode_1BYTE_DATA(name_attr), 16) == 0);

    Py_DECREF(module_attr);
    Py_DECREF(name_attr);
    Py_DECREF(class_attr);
  }
  state_estimator_msgs__msg__ContactDetection * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // stance_lf
    PyObject * field = PyObject_GetAttrString(_pymsg, "stance_lf");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->stance_lf = (Py_True == field);
    Py_DECREF(field);
  }
  {  // stance_rf
    PyObject * field = PyObject_GetAttrString(_pymsg, "stance_rf");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->stance_rf = (Py_True == field);
    Py_DECREF(field);
  }
  {  // stance_lh
    PyObject * field = PyObject_GetAttrString(_pymsg, "stance_lh");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->stance_lh = (Py_True == field);
    Py_DECREF(field);
  }
  {  // stance_rh
    PyObject * field = PyObject_GetAttrString(_pymsg, "stance_rh");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->stance_rh = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * state_estimator_msgs__msg__contact_detection__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ContactDetection */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("state_estimator_msgs.msg._contact_detection");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ContactDetection");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  state_estimator_msgs__msg__ContactDetection * ros_message = (state_estimator_msgs__msg__ContactDetection *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stance_lf
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->stance_lf ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "stance_lf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stance_rf
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->stance_rf ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "stance_rf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stance_lh
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->stance_lh ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "stance_lh", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stance_rh
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->stance_rh ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "stance_rh", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
