
#include <typeinfo>

std::type_info::~type_info() = default;

// As described in https://itanium-cxx-abi.github.io/cxx-abi/abi.html#rtti-layout
namespace __cxxabiv1
{

class __class_type_info : public std::type_info
{
public:
    explicit __class_type_info(const char* n) : type_info(n) {}
    virtual ~__class_type_info();
    virtual bool __do_catch(const type_info* thr_type, void** thr_obj, unsigned outer) const { return false; }
    virtual bool __do_upcast(const __class_type_info* dst_type, void** obj_ptr) const { return false; }
};

class __si_class_type_info : public __class_type_info
{
public:
    const __class_type_info* __base_type;
    explicit __si_class_type_info(const char* n, const __class_type_info* base) 
        : __class_type_info(n), __base_type(base) {}
    virtual ~__si_class_type_info();
};

class __vmi_class_type_info : public __class_type_info
{
public:
    unsigned int __flags;
    unsigned int __base_count;
    explicit __vmi_class_type_info(const char* n, int flags) 
        : __class_type_info(n), __flags(flags), __base_count(0) {}
    virtual ~__vmi_class_type_info();
};

// Explicitly default-define the destructors here so the vtables can be generated in this translation unit.
__class_type_info    ::~__class_type_info    () = default;
__si_class_type_info ::~__si_class_type_info () = default;
__vmi_class_type_info::~__vmi_class_type_info() = default;

}
// namespace __cxxabiv1
