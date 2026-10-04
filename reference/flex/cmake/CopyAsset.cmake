# copy asset file from source directory to binary directory.
function(copy_assets)
  set(oneValueArgs TARGET DESTINATION)
  set(multiValueArgs SOURCES)
  cmake_parse_arguments(asset "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  set(copied_files "")

  foreach(asset ${asset_SOURCES})
    get_filename_component(asset_fname ${asset} NAME)
    get_filename_component(asset_fpath ${asset} ABSOLUTE)
    set_source_files_properties(${asset} PROPERTIES HEADER_FILE_ONLY TRUE)

    set(output_file ${asset_DESTINATION}/${asset_fname})

    add_custom_command(
      OUTPUT ${output_file}
      COMMAND ${CMAKE_COMMAND} -E copy_if_different "${asset_fpath}" "${output_file}"
      DEPENDS "${asset_fpath}"
    )
    list(APPEND copied_files "${output_file}")
  endforeach()

  add_custom_target(
    ${asset_TARGET}
    ALL
    DEPENDS ${copied_files}
  )
endfunction()