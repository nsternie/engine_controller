
class MathHelper:

    @staticmethod
    def mapValue(value, leftMin, leftMax, rightMin, rightMax):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        if value > leftMin and value < leftMax:
            # Convert the left range into a 0-1 range (float)
            valueScaled = float(value - leftMin) / float(leftSpan)
        elif value >= leftMax:
            valueScaled = 1
        elif value <= leftMin:
            valueScaled = 0

        # Convert the 0-1 range into a value in the right range.
        return rightMin + (valueScaled * rightSpan)