
#include <typeinfo>

std::type_info::~type_info() = default;

// As described in https://itanium-cxx-abi.github.io/cxx-abi/abi.html#rtti-layout
namespace __cxxabiv1
{

class __class_type_info : public std::type_info
{
public:
    ~__class_type_info() override;
};

class __si_class_type_info : public __class_type_info
{
public:
    const __class_type_info *__base_type;

    ~__si_class_type_info() override;
};

// Explicitly default-define the destructors here so the vtables can be generated in this translation unit.
__class_type_info   ::~__class_type_info   () = default;
__si_class_type_info::~__si_class_type_info() = default;

}
// namespace __cxxabiv1
