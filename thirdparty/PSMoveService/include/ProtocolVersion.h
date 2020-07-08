#ifndef PROTOCOL_VERSION_H
#define PROTOCOL_VERSION_H

/// Conventional string-ification macro.
// From: http://stackoverflow.com/questions/5256313/c-c-macro-string-concatenation
#if !defined(PSM_STRINGIZE)
    #define PSM_STRINGIZEIMPL(x) #x
    #define PSM_STRINGIZE(x)     PSM_STRINGIZEIMPL(x)
#endif

// Current version of this release
#define PSM_RELEASE_VERSION_PRODUCT 0
#define PSM_RELEASE_VERSION_MAJOR   9
#define PSM_RELEASE_VERSION_PHASE   alpha
#define PSM_RELEASE_VERSION_MINOR   9
#define PSM_RELEASE_VERSION_RELEASE 0
#define PSM_RELEASE_VERSION_HOTFIX  1

/// "Product.Major-Phase Minor.Release.Hotfix"
#if !defined(PSM_RELEASE_VERSION_STRING)
    #define PSM_RELEASE_VERSION_STRING PSM_STRINGIZE(PSM_RELEASE_VERSION_PRODUCT.PSM_RELEASE_VERSION_MAJOR-PSM_RELEASE_VERSION_PHASE PSM_RELEASE_VERSION_MINOR.PSM_RELEASE_VERSION_RELEASE.PSM_RELEASE_VERSION_HOTFIX)
#endif

// Version of the release that the protocol comes from (<= current release)
#define PSM_PROTOCOL_VERSION_PRODUCT 0
#define PSM_PROTOCOL_VERSION_MAJOR   9
#define PSM_PROTOCOL_VERSION_PHASE   alpha
#define PSM_PROTOCOL_VERSION_MINOR   9
#define PSM_PROTOCOL_VERSION_RELEASE 0
#define PSM_PROTOCOL_VERSION_HOTFIX  0

/// "Product.Major-Phase Minor.Release.Hotfix"
#if !defined(PSM_PROTOCOL_VERSION_STRING)
    #define PSM_PROTOCOL_VERSION_STRING PSM_STRINGIZE(PSM_PROTOCOL_VERSION_PRODUCT.PSM_PROTOCOL_VERSION_MAJOR-PSM_PROTOCOL_VERSION_PHASE PSM_PROTOCOL_VERSION_MINOR.PSM_PROTOCOL_VERSION_RELEASE.PSM_PROTOCOL_VERSION_HOTFIX)
#endif

#endif // PROTOCOL_VERSION_H