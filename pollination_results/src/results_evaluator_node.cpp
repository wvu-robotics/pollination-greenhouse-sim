#include <pollination_results/results_evaluator.hpp>

int main(int argc, char**argv)
{
    ros::init(argc, argv, "results_evaluator_node");
    ResultsEvaluator results_evaluator;
    results_evaluator.run();
    return 0;
}
