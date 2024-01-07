from networktables import NetworkTables

robot_ip = ""

NetworkTables.initialize(robot_ip)

table_name = "AprilTags"

table = NetworkTables.getTable(table_name)


def insertCoords(x, y, h):
    table.putNumber("X", x)
    table.putNumber("Y", y)
    table.putNumber("Heading", h)
