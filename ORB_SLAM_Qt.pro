QT += core
QT -= gui

TARGET = ORB_SLAM_Qt
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

INCLUDEPATH += \
    $$PWD/include \
    /opt/ros/jade/include \
    /usr/include/eigen3

SOURCES += \
    src/Converter.cc \
    src/Frame.cc \
    src/FramePublisher.cc \
    src/Initializer.cc \
    src/KeyFrame.cc \
    src/KeyFrameDatabase.cc \
    src/LocalMapping.cc \
    src/LoopClosing.cc \
    src/main.cc \
    src/Map.cc \
    src/MapPoint.cc \
    src/MapPublisher.cc \
    src/Optimizer.cc \
    src/ORBextractor.cc \
    src/ORBmatcher.cc \
    src/PnPsolver.cc \
    src/Sim3Solver.cc \
    src/Tracking.cc \
    Thirdparty/g2o/g2o/types/sba/types_six_dof_expmap.cpp \
    Thirdparty/g2o/g2o/core/batch_stats.cpp \
    Thirdparty/g2o/g2o/core/cache.cpp \
    Thirdparty/g2o/g2o/core/estimate_propagator.cpp \
    Thirdparty/g2o/g2o/core/factory.cpp \
    Thirdparty/g2o/g2o/core/hyper_dijkstra.cpp \
    Thirdparty/g2o/g2o/core/hyper_graph.cpp \
    Thirdparty/g2o/g2o/core/hyper_graph_action.cpp \
    Thirdparty/g2o/g2o/core/jacobian_workspace.cpp \
    Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.cpp \
    Thirdparty/g2o/g2o/core/matrix_structure.cpp \
    Thirdparty/g2o/g2o/core/optimizable_graph.cpp \
    Thirdparty/g2o/g2o/core/optimization_algorithm.cpp \
    Thirdparty/g2o/g2o/core/optimization_algorithm_dogleg.cpp \
    Thirdparty/g2o/g2o/core/optimization_algorithm_factory.cpp \
    Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.cpp \
    Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp \
    Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.cpp \
    Thirdparty/g2o/g2o/core/parameter.cpp \
    Thirdparty/g2o/g2o/core/parameter_container.cpp \
    Thirdparty/g2o/g2o/core/robust_kernel.cpp \
    Thirdparty/g2o/g2o/core/robust_kernel_factory.cpp \
    Thirdparty/g2o/g2o/core/robust_kernel_impl.cpp \
    Thirdparty/g2o/g2o/core/solver.cpp \
    Thirdparty/g2o/g2o/core/sparse_block_matrix_test.cpp \
    Thirdparty/g2o/g2o/core/sparse_optimizer.cpp \
    Thirdparty/g2o/g2o/core/sparse_optimizer_terminate_action.cpp \
    Thirdparty/g2o/g2o/solvers/cholmod/solver_cholmod.cpp \
    Thirdparty/g2o/g2o/solvers/dense/solver_dense.cpp \
    Thirdparty/g2o/g2o/stuff/command_args.cpp \
    Thirdparty/g2o/g2o/stuff/filesys_tools.cpp \
    Thirdparty/g2o/g2o/stuff/opengl_primitives.cpp \
    Thirdparty/g2o/g2o/stuff/property.cpp \
    Thirdparty/g2o/g2o/stuff/sampler.cpp \
    Thirdparty/g2o/g2o/stuff/sparse_helper.cpp \
    Thirdparty/g2o/g2o/stuff/string_tools.cpp \
    Thirdparty/g2o/g2o/stuff/tictoc.cpp \
    Thirdparty/g2o/g2o/stuff/timeutil.cpp \
    Thirdparty/g2o/g2o/stuff/os_specific.c \
    Thirdparty/g2o/g2o/types/sba/types_sba.cpp \
    Thirdparty/g2o/g2o/types/sim3/types_seven_dof_expmap.cpp \
    Thirdparty/g2o/g2o/types/slam3d/dquat2mat.cpp \
    Thirdparty/g2o/g2o/types/slam3d/dquat2mat_maxima_generated.cpp \
    Thirdparty/g2o/g2o/types/slam3d/edge_se3.cpp \
    Thirdparty/g2o/g2o/types/slam3d/edge_se3_offset.cpp \
    Thirdparty/g2o/g2o/types/slam3d/edge_se3_pointxyz.cpp \
    Thirdparty/g2o/g2o/types/slam3d/edge_se3_pointxyz_depth.cpp \
    Thirdparty/g2o/g2o/types/slam3d/edge_se3_pointxyz_disparity.cpp \
    Thirdparty/g2o/g2o/types/slam3d/edge_se3_prior.cpp \
    Thirdparty/g2o/g2o/types/slam3d/isometry3d_gradients.cpp \
    Thirdparty/g2o/g2o/types/slam3d/isometry3d_mappings.cpp \
    Thirdparty/g2o/g2o/types/slam3d/parameter_camera.cpp \
    Thirdparty/g2o/g2o/types/slam3d/parameter_se3_offset.cpp \
    Thirdparty/g2o/g2o/types/slam3d/parameter_stereo_camera.cpp \
    Thirdparty/g2o/g2o/types/slam3d/types_slam3d.cpp \
    Thirdparty/g2o/g2o/types/slam3d/vertex_pointxyz.cpp \
    Thirdparty/g2o/g2o/types/slam3d/vertex_se3.cpp

HEADERS += \
    include/Converter.h \
    include/Frame.h \
    include/FramePublisher.h \
    include/Initializer.h \
    include/KeyFrame.h \
    include/KeyFrameDatabase.h \
    include/LocalMapping.h \
    include/LoopClosing.h \
    include/Map.h \
    include/MapPoint.h \
    include/MapPublisher.h \
    include/Optimizer.h \
    include/ORBextractor.h \
    include/ORBmatcher.h \
    include/ORBVocabulary.h \
    include/PnPsolver.h \
    include/Sim3Solver.h \
    include/Tracking.h \
    Thirdparty/g2o/g2o/core/base_binary_edge.h \
    Thirdparty/g2o/g2o/core/base_binary_edge.hpp \
    Thirdparty/g2o/g2o/core/base_edge.h \
    Thirdparty/g2o/g2o/core/base_multi_edge.h \
    Thirdparty/g2o/g2o/core/base_multi_edge.hpp \
    Thirdparty/g2o/g2o/core/base_unary_edge.h \
    Thirdparty/g2o/g2o/core/base_unary_edge.hpp \
    Thirdparty/g2o/g2o/core/base_vertex.h \
    Thirdparty/g2o/g2o/core/base_vertex.hpp \
    Thirdparty/g2o/g2o/core/batch_stats.h \
    Thirdparty/g2o/g2o/core/block_solver.h \
    Thirdparty/g2o/g2o/core/block_solver.hpp \
    Thirdparty/g2o/g2o/core/cache.h \
    Thirdparty/g2o/g2o/core/creators.h \
    Thirdparty/g2o/g2o/core/estimate_propagator.h \
    Thirdparty/g2o/g2o/core/factory.h \
    Thirdparty/g2o/g2o/core/g2o_core_api.h \
    Thirdparty/g2o/g2o/core/hyper_dijkstra.h \
    Thirdparty/g2o/g2o/core/hyper_graph.h \
    Thirdparty/g2o/g2o/core/hyper_graph_action.h \
    Thirdparty/g2o/g2o/core/jacobian_workspace.h \
    Thirdparty/g2o/g2o/core/linear_solver.h \
    Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.h \
    Thirdparty/g2o/g2o/core/matrix_operations.h \
    Thirdparty/g2o/g2o/core/matrix_structure.h \
    Thirdparty/g2o/g2o/core/openmp_mutex.h \
    Thirdparty/g2o/g2o/core/optimizable_graph.h \
    Thirdparty/g2o/g2o/core/optimization_algorithm.h \
    Thirdparty/g2o/g2o/core/optimization_algorithm_dogleg.h \
    Thirdparty/g2o/g2o/core/optimization_algorithm_factory.h \
    Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h \
    Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h \
    Thirdparty/g2o/g2o/core/optimization_algorithm_property.h \
    Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.h \
    Thirdparty/g2o/g2o/core/parameter.h \
    Thirdparty/g2o/g2o/core/parameter_container.h \
    Thirdparty/g2o/g2o/core/robust_kernel.h \
    Thirdparty/g2o/g2o/core/robust_kernel_factory.h \
    Thirdparty/g2o/g2o/core/robust_kernel_impl.h \
    Thirdparty/g2o/g2o/core/solver.h \
    Thirdparty/g2o/g2o/core/sparse_block_matrix.h \
    Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp \
    Thirdparty/g2o/g2o/core/sparse_block_matrix_ccs.h \
    Thirdparty/g2o/g2o/core/sparse_block_matrix_diagonal.h \
    Thirdparty/g2o/g2o/core/sparse_optimizer.h \
    Thirdparty/g2o/g2o/core/sparse_optimizer_terminate_action.h \
    Thirdparty/g2o/g2o/solvers/cholmod/linear_solver_cholmod.h \
    Thirdparty/g2o/g2o/solvers/dense/linear_solver_dense.h \
    Thirdparty/g2o/g2o/stuff/color_macros.h \
    Thirdparty/g2o/g2o/stuff/command_args.h \
    Thirdparty/g2o/g2o/stuff/filesys_tools.h \
    Thirdparty/g2o/g2o/stuff/g2o_stuff_api.h \
    Thirdparty/g2o/g2o/stuff/macros.h \
    Thirdparty/g2o/g2o/stuff/misc.h \
    Thirdparty/g2o/g2o/stuff/opengl_primitives.h \
    Thirdparty/g2o/g2o/stuff/opengl_wrapper.h \
    Thirdparty/g2o/g2o/stuff/os_specific.h \
    Thirdparty/g2o/g2o/stuff/property.h \
    Thirdparty/g2o/g2o/stuff/sampler.h \
    Thirdparty/g2o/g2o/stuff/scoped_pointer.h \
    Thirdparty/g2o/g2o/stuff/sparse_helper.h \
    Thirdparty/g2o/g2o/stuff/string_tools.h \
    Thirdparty/g2o/g2o/stuff/tictoc.h \
    Thirdparty/g2o/g2o/stuff/timeutil.h \
    Thirdparty/g2o/g2o/stuff/unscented.h \
    Thirdparty/g2o/g2o/types/sba/g2o_types_sba_api.h \
    Thirdparty/g2o/g2o/types/sba/sbacam.h \
    Thirdparty/g2o/g2o/types/sba/types_sba.h \
    Thirdparty/g2o/g2o/types/sba/types_six_dof_expmap.h \
    Thirdparty/g2o/g2o/types/sim3/sim3.h \
    Thirdparty/g2o/g2o/types/sim3/types_seven_dof_expmap.h \
    Thirdparty/g2o/g2o/types/slam3d/dquat2mat.h \
    Thirdparty/g2o/g2o/types/slam3d/edge_se3.h \
    Thirdparty/g2o/g2o/types/slam3d/edge_se3_offset.h \
    Thirdparty/g2o/g2o/types/slam3d/edge_se3_pointxyz.h \
    Thirdparty/g2o/g2o/types/slam3d/edge_se3_pointxyz_depth.h \
    Thirdparty/g2o/g2o/types/slam3d/edge_se3_pointxyz_disparity.h \
    Thirdparty/g2o/g2o/types/slam3d/edge_se3_prior.h \
    Thirdparty/g2o/g2o/types/slam3d/g2o_types_slam3d_api.h \
    Thirdparty/g2o/g2o/types/slam3d/isometry3d_gradients.h \
    Thirdparty/g2o/g2o/types/slam3d/isometry3d_mappings.h \
    Thirdparty/g2o/g2o/types/slam3d/parameter_camera.h \
    Thirdparty/g2o/g2o/types/slam3d/parameter_se3_offset.h \
    Thirdparty/g2o/g2o/types/slam3d/parameter_stereo_camera.h \
    Thirdparty/g2o/g2o/types/slam3d/se3_ops.h \
    Thirdparty/g2o/g2o/types/slam3d/se3_ops.hpp \
    Thirdparty/g2o/g2o/types/slam3d/se3quat.h \
    Thirdparty/g2o/g2o/types/slam3d/types_slam3d.h \
    Thirdparty/g2o/g2o/types/slam3d/vertex_pointxyz.h \
    Thirdparty/g2o/g2o/types/slam3d/vertex_se3.h

OTHER_FILES += \
    Thirdparty/g2o/g2o/core/CMakeLists.txt \
    Thirdparty/g2o/g2o/types/slam3d/dquat2mat.wxm

