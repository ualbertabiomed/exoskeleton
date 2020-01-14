"""
Conversion functions needed for communincation between nodes
"""

# Number of ticks in a full circle
CONVERSION_FACTOR = 8192 / 360

def convert_counts_to_angle(counts):
    return counts / CONVERSION_FACTOR

def convert_angle_to_count(angle):
    return angle * CONVERSION_FACTOR
