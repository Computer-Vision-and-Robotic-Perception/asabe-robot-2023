/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef STEPPER_HARDWARE__VISIBLITY_CONTROL_H_
#define STEPPER_HARDWARE__VISIBLITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define STEPPER_HARDWARE_EXPORT __attribute__((dllexport))
#define STEPPER_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define STEPPER_HARDWARE_EXPORT __declspec(dllexport)
#define STEPPER_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef STEPPER_HARDWARE_BUILDING_DLL
#define STEPPER_HARDWARE_PUBLIC STEPPER_HARDWARE_EXPORT
#else
#define STEPPER_HARDWARE_PUBLIC STEPPER_HARDWARE_IMPORT
#endif
#define STEPPER_HARDWARE_PUBLIC_TYPE STEPPER_HARDWARE_PUBLIC
#define STEPPER_HARDWARE_LOCAL
#else
#define STEPPER_HARDWARE_EXPORT __attribute__((visibility("default")))
#define STEPPER_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define STEPPER_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define STEPPER_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define STEPPER_HARDWARE_PUBLIC
#define STEPPER_HARDWARE_LOCAL
#endif
#define STEPPER_HARDWARE_PUBLIC_TYPE
#endif

#endif  // STEPPER_HARDWARE__VISIBLITY_CONTROL_H_
