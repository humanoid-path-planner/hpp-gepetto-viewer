/// \mainpage
///
/// \section hpp_gepetto_viewer_introduction Introduction
///
/// This package implements simultaneous control to hppcorbaserver and
/// gepetto-viewer-server.
///
/// The main python classes are
/// \li gepetto.viewer.Viewer that implements a simultaneous control to
///     hppcorbaserver and gepetto-viewer-server,
/// \li gepetto.manipulation.viewer.Viewer that implements a simultaneous
///     control to hpp-manipulation-server and gepetto-viewer-server,
/// \li gepetto.viewer_factory.ViewerFactory that implements a wrapper
///     that stores commands to be sent to gepetto-viewer-server and
///     creates new clients when gepetto-viewer-server is restarted,
/// \li gepetto.manipulation.viewer_factory.ViewerFactory that
///     implements a viewer factory for hpp-manipulation-server,
/// \li gepetto.path_player.PathPlayer that displays a path by sampling
///     configurations along the path.
