"""
This file constains constants that are used across various components of the 
autonomy stack.
"""

"""
The location of the data file containing the LQR gain for level, stable flight.
"""
const LQR_gain_file = joinpath(@__DIR__, "..", "data", "lqr_gain.json")