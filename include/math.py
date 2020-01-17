"""
Conversion functions needed for communincation between nodes
"""
# Return type FLOAT
def convert_counts_to_angle(counts):
	conversion_factor = float(8192/360)
	angle = float(counts / conversion_factor)
	return angle

# Return type FLOAT
def convert_angle_to_count(angle)
	conversion_factor = float(8192/360)
	counts = float(angle * conversion_factor)
	return counts
