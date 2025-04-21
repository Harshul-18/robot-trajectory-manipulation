import pandas as pd
import os
from scipy.stats import f_oneway
from statsmodels.stats.multicomp import pairwise_tukeyhsd

paths = {
	'graphs_1/pid_metrics.csv': (0.5, 0.5),
	'graphs_2/pid_metrics.csv': (1, 0),
	'graphs_3/pid_metrics.csv': (1, 0.5),
	'graphs_4/pid_metrics.csv': (0.5, 0.75)
}

controller_name_mapping = {
	'pid_normal': 'PID',
	'pid_fractional': 'Fractional PID',
	'pid_adaptive_fractional': 'Adaptive Fractional PID',
	'pid_nonlinear_fractional': 'Non-Linear Sigmoid Fractional PID',
	'pid_time_delay_fractional': 'Time Delay Fractional PID'
}

merged_data = pd.DataFrame()

for path, final_position in paths.items():
	data = pd.read_csv(path)
	data['Final Position X'] = final_position[0]
	data['Final Position Y'] = final_position[1]
	merged_data = pd.concat([merged_data, data], ignore_index=True)

controller_order = ['pid_normal', 'pid_fractional', 'pid_adaptive_fractional', 'pid_nonlinear_fractional', 'pid_time_delay_fractional']
merged_data['Controller'] = pd.Categorical(merged_data['Controller'], categories=controller_order, ordered=True)
merged_data = merged_data.sort_values('Controller').reset_index(drop=True)

merged_data = merged_data[['Controller', 'Final Position X', 'Final Position Y', 'Error Area', 'Points Traversed', 'Sum of Errors', 'Time Taken']]
merged_data['Controller'] = merged_data['Controller'].map(controller_name_mapping)

merged_data.to_csv('final_pid_metrics.csv', index=False)

print("Final PID results saved")

anova_results = {}
metrics = ['Error Area', 'Points Traversed', 'Sum of Errors', 'Time Taken']

controller_order = [v for k, v in controller_name_mapping.items()]

for metric in metrics:
	groups = [merged_data[merged_data['Controller'] == controller][metric] for controller in controller_order]
	f_val, p_val = f_oneway(*groups)
	anova_results[metric] = {'F-Statistic': f_val, 'p-Value': p_val}

anova_df = pd.DataFrame(anova_results).T
print('ANOVA test results:')
print(anova_df)

anova_df.to_csv('anova_results.csv', index=True)

print('ANOVA results saved to "anova_results.csv".')

tukey = pairwise_tukeyhsd(endog=merged_data['Error Area'], groups=merged_data['Controller'], alpha=0.05)
tukey_df = pd.DataFrame(data=tukey.summary())

print("Tukey's HSD test results for Error Area:")
print(tukey_df)

tukey_df.to_csv("tukey_hsd_results.csv", index=False)

print("Tukey's HSD results saved to 'tukey_hsd_results.csv'")















