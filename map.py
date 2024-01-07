class Tag:
    def __init__(self, id, x, y, rotation):
        self.id = id
        self.x = x
        self.y = y
        self.rotation = rotation


class Map:
    def __init__(self):
        self.tags = []
        pass

    def set_tag(self, id, x, y, rotation):
        """
        Adds a tag to the map
        :param id: id of the tag
        :param x: x (long side) of the tag
        :param y: y (short side) of the tag
        :param rotation: 0 is right, pi/2 is down, pi is left, 3pi/2 is up
        :return: None
        """
        self.tags.append(Tag(id, x, y, rotation))

    def get_tag_info(self, id):
        """
        Gets the tag info
        :param id: id of the tag
        :return: Tag object
        """
        for tag in self.tags:
            if tag.id == id:
                return tag
        return None

    def get_position(self, tag_ids, orientations):
        """
        Gets the position of the robot
        :param tag_ids: list of tag ids
        :return: x, y, rotation
        """
        tags = []
        for id in tag_ids:
            tags.append(self.get_tag_info(id))

        for i in range(len(orientations)):
            o = orientations[i]
            t = tags[i]
            rotation = o[0]
            translation = o[1]
            # TODO: calculate the position of the robot
            """
            We need the camera position on the robot (relative to the center of the robot)
            Then we can translate the april tag position to the robot position (using the rotation and translation)
            Then we can translate the robot position to the field position (using the rotation and translation)
            """