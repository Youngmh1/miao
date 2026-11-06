add_library(miao.core SHARED
        graph/graph.cc
        graph/optimizer.cc

        common/string_tools.cc

        math/marginal_covariance_cholesky.cc

        opti_algo/optimization_algorithm.cc
        opti_algo/opti_algo_gauss_newton.cc
        opti_algo/opti_algo_lm.cc
        opti_algo/opti_algo_dogleg.cc

        solver/solver.cc
)

target_include_directories(miao.core PUBLIC
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/deps
)

target_link_libraries(
        miao.core
        glog tbb
)

install(TARGETS miao.core
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)