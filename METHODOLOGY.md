# Methodology

Replace this template with your own. Keep it short: write what a teammate would need to run your work and trust it.

## 1. How to run it

1. Make sure you are in the embedded-take-home folder.
2. Go to Extensions and install Python.
3. Run the program using "python sim/launch.py --visualize" in the terminal.

## 2. Thought process

Your approach and the reasoning behind it, in bullet points.

- I initially used two GPS readings to make a calculation for the heading.
- I switched to having a list of GPS readings instead where I calculated the average position to get a heading from.

## 3. Known limitations

What doesn't work:
- Covariance is not utilized at all in fusion weighting. w_gps = .2, which is a fixed weighting. 
- At times, the heading will be highly innaccurate due to the average position of the past gps readings being skewed heavily by outliers. Increasing the length of the list would make the heading more consistent but less responsive to sudden turns.

What I would do next:
- I would use proper confidence-weighted fusion where the fusion weighting is dynamic. For example, if no gps reading is received within a certain amount of time, I could make the weight rely a lot more heavily on the encoders.

