#-----------------------------------------------------------------------------
set(MODULE_NAME navigation)

#-----------------------------------------------------------------------------
set(MODULE_PYTHON_SCRIPTS 
${MODULE_NAME}.py
${MODULE_NAME}Lib/__init__.py
${MODULE_NAME}Lib/niiCook.py
${MODULE_NAME}Lib/Rendering.py
${MODULE_NAME}Lib/simulationFunction.py
${MODULE_NAME}Lib/helpFunction.py
)

# set(MODULE_PYTHON_SCRIPTS helpFunction.py)
# set(MODULE_PYTHON_SCRIPTS niiCook.py)
# set(MODULE_PYTHON_SCRIPTS Rendering.py)
# set(MODULE_PYTHON_SCRIPTS simulationFunction.py)

set(MODULE_PYTHON_RESOURCES
  Resources/Icons/${MODULE_NAME}.png
  Resources/UI/${MODULE_NAME}.ui
  Resources/model/NeedleModel.stl
  )

#-----------------------------------------------------------------------------
slicerMacroBuildScriptedModule(
  NAME ${MODULE_NAME}
  SCRIPTS ${MODULE_PYTHON_SCRIPTS}
  RESOURCES ${MODULE_PYTHON_RESOURCES}
  WITH_GENERIC_TESTS
  )

#-----------------------------------------------------------------------------
if(BUILD_TESTING)

  # Register the unittest subclass in the main script as a ctest.
  # Note that the test will also be available at runtime.
  slicer_add_python_unittest(SCRIPT ${MODULE_NAME}.py)


  # Additional build-time testing
  add_subdirectory(Testing)
endif()
