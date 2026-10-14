# One reading of the scintillator: counts per second over one counting interval.
# Published on /scintillator/count_rate by the scintillator driver (docs/scintillator.md).

# stamp = END of the counting interval (Pi time); frame_id is not used.
std_msgs/Header header

# Counts per second, averaged over the interval.
float64 counts_per_second

# Length of the counting interval in seconds (about 1.0). The reading belongs to [stamp - interval_s, stamp].
float64 interval_s
