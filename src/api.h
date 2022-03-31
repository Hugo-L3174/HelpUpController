#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define HelpUpController_DLLIMPORT __declspec(dllimport)
#  define HelpUpController_DLLEXPORT __declspec(dllexport)
#  define HelpUpController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define HelpUpController_DLLIMPORT __attribute__((visibility("default")))
#    define HelpUpController_DLLEXPORT __attribute__((visibility("default")))
#    define HelpUpController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define HelpUpController_DLLIMPORT
#    define HelpUpController_DLLEXPORT
#    define HelpUpController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef HelpUpController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define HelpUpController_DLLAPI
#  define HelpUpController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef HelpUpController_EXPORTS
#    define HelpUpController_DLLAPI HelpUpController_DLLEXPORT
#  else
#    define HelpUpController_DLLAPI HelpUpController_DLLIMPORT
#  endif // HelpUpController_EXPORTS
#  define HelpUpController_LOCAL HelpUpController_DLLLOCAL
#endif // HelpUpController_STATIC