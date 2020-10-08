import unittest
import numpy
import math

gravity = numpy.array([0, 0, -9.80665])

class LinkBuilder():
    """
        For building up a robotic object for gravity compensation calculations
        used like:

        b = LinkBuilder()
        b.addLink(Link(1, 2, 3, 4, (1,1,1), 1))
        b.addLink(Link(2, 3, 4, 5, (1,1,1), 1))
        b.addLink(Link(3, 4, 5, 6, (1,1,1), 1))
        links = b.build()

        addLink returns the builder for chaining
    """
    def __init__(self):
        self.links = None

    def addLink(self, link):
        if self.links is None:
            self.links = link
        else:
            self.links.addLinkToEnd(link)
        return self

    def build(self):
        return self.links

class Link():
    """
        Uses DH parameters
        https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters to
        represent the location of each link

        Center of mass is relative to the coordinates of the DH frame
        all values are in SI units - I.E. m, kg, and radians
    """
    def __init__(self, d, theta, r, alpha, centerOfMass, mass):
        self.d = d          # offset along previous z to the common normal # in meters
        self.theta = theta  # angle about previous z, from old x to new x  # in radians
        self.r = r          # length of the common normal. Assuming a revolute joint, this is the radius about previous z (Sometimes called a) # in meters
        self.alpha = alpha  # angle about common normal, from old z axis to new z axis # in radians
        self.com = centerOfMass # tuple (x, y, z), in meters
        self.mass = mass    # in kilograms
        self.nextLink = None

    def __repr__(self):
        return str(self)

    def __str__(self):
        return "Link {{{}kg at {}}} [d: {}, \u03B8: {}, r: {}, \u03B1: {}] --> {}".format(
                self.mass, self.com, self.d, self.theta, self.r, self.alpha, str(self.nextLink))

    def __len__(self):
        if self.nextLink is not None:
            return len(self.nextLink) + 1
        return 1

    def addLinkToEnd(self, link):
        if self.nextLink is None:
            self.nextLink = link
        else:
            self.nextLink.addLinkToEnd(link)

    def calculateInverseDynamics(self, *angles):
        """
            Calculates the required torque for each motor to compensate
            for gravity. Uses DH matrices to calculate the position on
            the forward recursive step, then uses Newton-Euler laws of
            motion to calculate the required moment acting on each motor
        """
        reqArgs = len(self)
        # https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
        if len(angles) == reqArgs:
            res = [None for _ in angles] + [(0, 0, 0)]
            self._calculateInverseDynamics(angles, 0, res)
            res.pop()
            return [tuple(x) for x in res]
        raise TypeError("calculateInverseDynamics on this object requires exactly {} arguments. ({} given)".format(reqArgs, len(angles)))

    def _calculateInverseDynamics(self, angles, index, moments):
        # calculate position of center of mass and joint origin taking into account rotation
        # TODO: use DH-matrix to calculate the absolute positions
        # Currently it doesn't do this, and will only work for 1 link sytems
        jointLoc = numpy.array([0,0,0])
        com = numpy.array(self.com)
        # call _calculateInverseDynamics on self.nextLink, incrementing index if has next
        if self.nextLink is not None:
            self.nextLink._calculateInverseDynamics(angles, index+1, moments)
        # calculate moment caused by gravity, insert into moments[index], taking into account moments[index+1]
        gravitationalForce = numpy.multiply(gravity, self.mass)
        radius = numpy.subtract(jointLoc, com)
        moment = numpy.cross(radius, gravitationalForce)
        moment = numpy.add(moment, moments[index+1])
        moments[index] = moment


def createTestData1():
    b = LinkBuilder()
    (
        b.addLink(Link(1, 2, 3, 4, (1,1,1), 1))
         .addLink(Link(2, 3, 4, 5, (1,1,1), 1))
         .addLink(Link(3, 4, 5, 6, (1,1,1), 1))
    )

    return b.build()

class TestGravityCompensation(unittest.TestCase):
    def compare_moments(self, moment1, moment2):
        self.assertEqual(len(moment1), len(moment2))
        for i in range(len(moment1)):
            self.assertTupleEqual(moment1[i], moment2[i])


    def test_link_to_string(self):
        l = createTestData1()
        self.assertEqual(str(l), "Link {1kg at (1, 1, 1)} [d: 1, θ: 2, r: 3, α: 4] --> Link {1kg at (1, 1, 1)} [d: 2, θ: 3, r: 4, α: 5] --> Link {1kg at (1, 1, 1)} [d: 3, θ: 4, r: 5, α: 6] --> None")

    def test_link_length(self):
        l = createTestData1()
        self.assertEqual(len(l), 3)
        l.addLinkToEnd(Link(1,1,1,1,(1,1,1),1))
        self.assertEqual(len(l), 4)

    def test_link_constructor(self):
        l = Link(1, 2, 3, 4, (5, 6, 7), 8)
        self.assertEqual(1, l.d)
        self.assertEqual(2, l.theta)
        self.assertEqual(3, l.r)
        self.assertEqual(4, l.alpha)
        self.assertEqual((5, 6, 7), l.com)
        self.assertEqual(8, l.mass)
        self.assertEqual(None, l.nextLink)

    def test_link_linkage(self):
        l = Link(1, 2, 3, 4, (5, 6, 7), 8)
        l2 = Link(11, 12, 13, 14, (15, 16, 17), 18)
        l.addLinkToEnd(l2)
        self.assertEqual(11, l.nextLink.d)
        self.assertEqual(12, l.nextLink.theta)
        self.assertEqual(13, l.nextLink.r)
        self.assertEqual(14, l.nextLink.alpha)
        self.assertEqual((15, 16, 17), l.nextLink.com)
        self.assertEqual(18, l.nextLink.mass)
        self.assertEqual(None, l.nextLink.nextLink)

    def test_inverse_dynamics(self):
        l = Link(0, 0, 0, math.pi / 2, (0.25, 0, 0), 1)
        ans = l.calculateInverseDynamics(0)
        self.compare_moments([(0, -9.80665/4, 0)], ans)

    # TODO: add more tests here

