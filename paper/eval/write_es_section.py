"""Write paper/tex/es_section.tex from paper/data/es/es_results.json."""
import json, sys
r = json.load(open(sys.argv[1])); es, pid = r["es"], r["pid"]
L = r["log"]
# first logged generation at which validation return beats PID
cross = next((g for g in L if g["val_return"] > pid["return_mean"]), None)
def pct(x): return f"{100*x:.0f}\\%"
txt = rf"""
Training ran for {r['gens']} generations ({r['gens']*r['pairs']*2*r['eps_per']:,} episodes, at most
{r['phys_steps_upper']/1e9:.2f}\,billion physics ticks) in {r['wall_s']/60:.1f}\,min of wall-clock time
on the two vCPUs (Fig.~\ref{{fig:es}}).
{"The validation return first exceeded the PID baseline after " + str(cross['gen']) + " generations (" + f"{cross['wall_s']/60:.1f}" + r"\,min)." if cross else "The validation return did not exceed the PID baseline."}
Table~\ref{{tab:es}} compares the final policy with PID on 100 held-out seeds. We count an episode
as a success if the vehicle does not crash and its mean position error over the last 2\,s is below 0.25\,m.

\begin{{table}}[t]
\centering
\caption{{Position hold on 100 held-out seeds (random start offset and wind).
Tail error is the mean position error over the final 2\,s of an episode.}}
\label{{tab:es}}
\footnotesize
\begin{{tabular}}{{@{{}}lcccc@{{}}}}
\toprule
Controller & Return & Tail err.\ (median) & Success & Crash \\
\midrule
Cascaded PID & {pid['return_mean']:.3f} & {pid['tail_err_median']:.3f}\,m & {pct(pid['success_rate'])} & {pct(pid['crash_rate'])} \\
ES policy (NumPy MLP) & {es['return_mean']:.3f} & {es['tail_err_median']:.3f}\,m & {pct(es['success_rate'])} & {pct(es['crash_rate'])} \\
\bottomrule
\end{{tabular}}
\end{{table}}

\begin{{figure}}[t]
\centering
\includegraphics[width=\columnwidth]{{fig_es}}
\caption{{CPU-only evolution-strategies training on \skysim: mean validation return on 20
fixed seeds against wall-clock time, with the cascaded PID baseline for reference.}}
\label{{fig:es}}
\end{{figure}}

This experiment is not a claim that ES beats PID control in general. The PID gains were tuned by hand
in a few minutes, and the ES policy has access to the same state. The claim it supports is narrower:
a closed-loop flight policy can be trained from scratch against \skysim's full rotor model on a
commodity CPU in minutes, without a GPU or a deep-learning framework.
"""
open(sys.argv[2], "w").write(txt)
print(txt)
