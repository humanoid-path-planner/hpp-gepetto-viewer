{
  lib,
  cmake,
  gepetto-viewer-corba,
  hpp-corbaserver,
  libsForQt5,
  pkg-config,
  python3Packages,
}:

python3Packages.buildPythonPackage {
  pname = "hpp-gepetto-viewer";
  version = "5.0.0";
  pyproject = false;

  src = lib.fileset.toSource {
    root = ./.;
    fileset = lib.fileset.unions [
      ./CMakeLists.txt
      ./doc
      ./package.xml
      ./src
    ];
  };

  strictDeps = true;

  nativeBuildInputs = [
    cmake
    libsForQt5.wrapQtAppsHook
    pkg-config
  ];
  buildInputs = [
    libsForQt5.qtbase
  ];
  propagatedBuildInputs = [
    gepetto-viewer-corba
    hpp-corbaserver
  ];

  doCheck = true;

  #pythonImportsCheck = [ "hpp.gepetto" ];

  meta = {
    description = "Display of hpp robots and obstacles in gepetto-viewer";
    homepage = "https://github.com/humanoid-path-planner/hpp-gepetto-viewer";
    license = lib.licenses.bsd2;
    maintainers = [ lib.maintainers.nim65s ];
  };
}
