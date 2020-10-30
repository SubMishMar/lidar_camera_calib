# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(projection_CONFIG_INCLUDED)
  return()
endif()
set(projection_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(projection_SOURCE_PREFIX /home/usl/catkin_ws/src/pbpc-cal/projection)
  set(projection_DEVEL_PREFIX /home/usl/catkin_ws/src/pbpc-cal/projection/cmake-build-debug/devel)
  set(projection_INSTALL_PREFIX "")
  set(projection_PREFIX ${projection_DEVEL_PREFIX})
else()
  set(projection_SOURCE_PREFIX "")
  set(projection_DEVEL_PREFIX "")
  set(projection_INSTALL_PREFIX /usr/local)
  set(projection_PREFIX ${projection_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'projection' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(projection_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/usr/include/pcl-1.7;/usr/include/eigen3;/usr/include;/usr/include/ni;/usr/include/vtk-6.2;/usr/include/freetype2;/usr/include/x86_64-linux-gnu/freetype2;/usr/include/jsoncpp;/usr/lib/openmpi/include/openmpi/opal/mca/event/libevent2021/libevent;/usr/lib/openmpi/include/openmpi/opal/mca/event/libevent2021/libevent/include;/usr/lib/openmpi/include;/usr/lib/openmpi/include/openmpi;/usr/include/python2.7;/usr/include/x86_64-linux-gnu;/usr/include/hdf5/openmpi;/usr/include/libxml2;/usr/include/tcl " STREQUAL " ")
  set(projection_INCLUDE_DIRS "")
  set(_include_dirs "/usr/include/pcl-1.7;/usr/include/eigen3;/usr/include;/usr/include/ni;/usr/include/vtk-6.2;/usr/include/freetype2;/usr/include/x86_64-linux-gnu/freetype2;/usr/include/jsoncpp;/usr/lib/openmpi/include/openmpi/opal/mca/event/libevent2021/libevent;/usr/lib/openmpi/include/openmpi/opal/mca/event/libevent2021/libevent/include;/usr/lib/openmpi/include;/usr/lib/openmpi/include/openmpi;/usr/include/python2.7;/usr/include/x86_64-linux-gnu;/usr/include/hdf5/openmpi;/usr/include/libxml2;/usr/include/tcl")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'subodh <subodh@todo.todo>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${projection_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'projection' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'projection' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/usl/catkin_ws/src/pbpc-cal/projection/${idir}'.  ${_report}")
    endif()
    _list_append_unique(projection_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "optimized;/usr/lib/x86_64-linux-gnu/libpcl_common.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_common.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_kdtree.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_kdtree.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_octree.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_octree.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_search.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_search.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_io.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_io.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_filters.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_filters.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_features.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_features.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_segmentation.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_segmentation.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_surface.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_surface.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_registration.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_registration.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_recognition.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_recognition.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_keypoints.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_keypoints.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_visualization.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_visualization.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_people.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_people.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_outofcore.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_outofcore.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_tracking.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_tracking.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;-lpthread;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_iostreams.so;/usr/lib/x86_64-linux-gnu/libboost_serialization.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/x86_64-linux-gnu/libboost_atomic.so;/usr/lib/x86_64-linux-gnu/libboost_regex.so;optimized;/usr/lib/x86_64-linux-gnu/libqhull.so;debug;/usr/lib/x86_64-linux-gnu/libqhull.so;/usr/lib/libOpenNI.so;optimized;/usr/lib/x86_64-linux-gnu/libflann_cpp_s.a;debug;/usr/lib/x86_64-linux-gnu/libflann_cpp_s.a;/usr/lib/x86_64-linux-gnu/libvtkImagingStencil-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkCommonMath-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtksys-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkCommonMisc-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkCommonSystem-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkImagingCore-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersAMR-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersCore-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkParallelCore-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOLegacy-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOCore-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libz.so;/usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkImagingSources-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingCore-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkCommonColor-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkImagingFourier-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkalglib-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersSources-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOImage-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkDICOMParser-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkmetaio-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libjpeg.so;/usr/lib/x86_64-linux-gnu/libpng.so;/usr/lib/x86_64-linux-gnu/libtiff.so;/usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkImagingColor-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libfreetype.so;/usr/lib/x86_64-linux-gnu/libvtkftgl-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOParallelNetCDF-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkParallelMPI-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libnetcdf_c++.so;/usr/lib/x86_64-linux-gnu/libnetcdf.so;/usr/lib/x86_64-linux-gnu/hdf5/serial/lib/libhdf5.so;/usr/lib/x86_64-linux-gnu/libpthread.so;/usr/lib/x86_64-linux-gnu/libsz.so;/usr/lib/x86_64-linux-gnu/libdl.so;/usr/lib/x86_64-linux-gnu/libm.so;/usr/lib/x86_64-linux-gnu/hdf5/serial/lib/libhdf5_hl.so;/usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOLSDyna-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOXML-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOGeometry-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libjsoncpp.so;/usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libexpat.so;/usr/lib/x86_64-linux-gnu/libvtkLocalExample-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkInfovisCore-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkGeovisCore-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkInfovisLayout-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkViewsCore-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkproj4-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libpython2.7.so;/usr/lib/x86_64-linux-gnu/libvtkTestingGenericBridge-6.2.so.6.2.0;/usr/lib/libgl2ps.so;/usr/lib/x86_64-linux-gnu/libvtkverdict-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOMovie-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libtheoraenc.so;/usr/lib/x86_64-linux-gnu/libtheoradec.so;/usr/lib/x86_64-linux-gnu/libogg.so;/usr/lib/x86_64-linux-gnu/libvtkFiltersImaging-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOMINC-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkViewsQt-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkViewsInfovis-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkChartsCore-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingLabel-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingImage-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersFlowPaths-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkxdmf2-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libxml2.so;/usr/lib/x86_64-linux-gnu/libvtkFiltersReebGraph-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOXdmf2-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOAMR-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkImagingStatistics-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOParallel-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersParallel-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIONetCDF-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkexoIIc-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkGUISupportQtOpenGL-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOParallelLSDyna-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelGeometry-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkGUISupportQtWebkit-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOPLY-6.2.so.6.2.0;/usr/lib/libvtkWrappingTools-6.2.a;/usr/lib/x86_64-linux-gnu/libvtkFiltersHyperTree-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingVolumeOpenGL-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOExodus-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOPostgreSQL-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOSQL-6.2.so.6.2.0;sqlite3;/usr/lib/x86_64-linux-gnu/libvtkWrappingJava-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelFlowPaths-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelStatistics-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersProgrammable-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelImaging-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingParallelLIC-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingLIC-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkInteractionImage-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersPython-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkWrappingPython27Core-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOParallelExodus-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersGeneric-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOVideo-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingQt-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersTexture-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOInfovis-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkGUISupportQtSQL-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingFreeTypeOpenGL-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkInfovisBoostGraphAlgorithms-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingGL2PS-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOGeoJSON-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersVerdict-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkViewsGeovis-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOImport-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkTestingIOSQL-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkPythonInterpreter-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOODBC-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOEnSight-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOMySQL-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingMatplotlib-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkDomainsChemistry-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOExport-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelMPI-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOParallelXML-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkTestingRendering-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOMPIParallel-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkParallelMPI4Py-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersSMP-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersSelection-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOVPIC-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkVPIC-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkImagingMath-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkImagingMorphological-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingParallel-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingFreeTypeFontConfig-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOFFMPEG-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOMPIImage-6.2.so.6.2.0;/usr/lib/x86_64-linux-gnu/libvtkIOGDAL-6.2.so.6.2.0")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND projection_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND projection_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT projection_NUM_DUMMY_TARGETS)
      set(projection_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::projection::wrapped-linker-option${projection_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR projection_NUM_DUMMY_TARGETS "${projection_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::projection::wrapped-linker-option${projection_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND projection_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND projection_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND projection_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/usl/catkin_ws/src/pbpc-cal/projection/cmake-build-debug/devel/lib;/home/usl/catkin_ws/devel/lib;/opt/ros/kinetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(projection_LIBRARY_DIRS ${lib_path})
      list(APPEND projection_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'projection'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND projection_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(projection_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${projection_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 projection_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${projection_dep}_FOUND)
      find_package(${projection_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${projection_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(projection_INCLUDE_DIRS ${${projection_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(projection_LIBRARIES ${projection_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${projection_dep}_LIBRARIES})
  _list_append_deduplicate(projection_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(projection_LIBRARIES ${projection_LIBRARIES})

  _list_append_unique(projection_LIBRARY_DIRS ${${projection_dep}_LIBRARY_DIRS})
  list(APPEND projection_EXPORTED_TARGETS ${${projection_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${projection_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
