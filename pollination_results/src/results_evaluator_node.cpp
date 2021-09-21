#include <pollination_results/results_evaluator.hpp>

int main(int argc, char**argv)
{
    ros::init(argc, argv, "results_evaluator_node");
    ResultsEvaluator results_evaluator;
    ros::spin();
    return 0;
}
