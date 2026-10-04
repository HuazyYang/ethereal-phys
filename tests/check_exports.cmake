# Export check for ethereal_phys.dll, run by ctest:
#
#     cmake -DDUMPBIN=<dumpbin.exe> -DDLL=<ethereal_phys.dll> -P check_exports.cmake
#
# donut/nvrhi/doc/com-interface-abi.md, section 2.1: a DLL exports only extern "C" functions. The export table
# must list at least one function, every name must start with etherealPhys, and none may be C++-mangled
# (?...). EXPECTED, when given, is the exact list of names (separated by commas).

if(NOT DUMPBIN OR NOT DLL)
    message(FATAL_ERROR "check_exports.cmake: set DUMPBIN and DLL")
endif()

execute_process(
    COMMAND "${DUMPBIN}" /nologo /exports "${DLL}"
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
    ERROR_VARIABLE output_err)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "dumpbin /exports ${DLL} failed (${result}):\n${output}\n${output_err}")
endif()

# The export rows: "    ordinal hint RVA name", with an optional " = target" after the name.
string(REPLACE "\r" "" output "${output}")
string(REPLACE ";" "@" output "${output}")
string(REPLACE "\n" ";" lines "${output}")

set(names "")
set(violations "")
foreach(line IN LISTS lines)
    if(line MATCHES "^ +[0-9]+ +[0-9A-Fa-f]+ +[0-9A-Fa-f]+ +([^ ]+)")
        set(name "${CMAKE_MATCH_1}")
        list(APPEND names "${name}")
        if(name MATCHES "^\\?" OR name MATCHES "^_Z")
            list(APPEND violations "C++-mangled export: ${name}")
        elseif(NOT name MATCHES "^etherealPhys[A-Z][A-Za-z0-9]*$")
            list(APPEND violations "export not named etherealPhys*: ${name}")
        endif()
    endif()
endforeach()

list(LENGTH names count)
if(count EQUAL 0)
    list(APPEND violations "the export table is empty (or dumpbin's output was not understood)")
endif()

if(EXPECTED)
    string(REPLACE "," ";" expected "${EXPECTED}")
    list(SORT expected)
    set(sorted ${names})
    list(SORT sorted)
    if(NOT "${sorted}" STREQUAL "${expected}")
        list(APPEND violations "the exports are [${sorted}], expected [${expected}]")
    endif()
endif()

if(violations)
    list(JOIN violations "\n  " report)
    message(FATAL_ERROR "Export check of ${DLL}: violations found:\n  ${report}\n\n${output}")
endif()
list(JOIN names ", " report)
message(STATUS "Export check: ${count} export(s), all etherealPhys*: ${report}")
