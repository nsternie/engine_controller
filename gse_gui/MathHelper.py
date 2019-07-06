
"""
Helper class with various mth functions
"""
class MathHelper:

    @staticmethod
    def mapValue(value, leftMin, leftMax, rightMin, rightMax):
        """
        Maps value from one range to a value on a new range

        :param value: value to map
        :param leftMin: minimum value on initial range
        :param leftMax: maximum value on initial range
        :param rightMin: minimum value on final range
        :param rightMax: maximum value on final range

        :returns new value on final range
        """
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Make sure initial value is within initial range
        # Otherwise return min or max final value
        if value > leftMin and value < leftMax:
            # Convert the left range into a 0-1 range (float)
            valueScaled = float(value - leftMin) / float(leftSpan)
        elif value >= leftMax:
            valueScaled = 1
        elif value <= leftMin:
            valueScaled = 0

        # Convert the 0-1 range into a value in the right range.
        return rightMin + (valueScaled * rightSpan)