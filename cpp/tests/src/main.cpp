#include "test_detectors.hpp"
#include "benchmark_detectors.hpp"

int main(int argc, char** argv)
{
    if(testDetectors() != 0)
        return -1;
    if(benchmarkDetectors() != 0)
        return -1;
    
    return 0;
}
