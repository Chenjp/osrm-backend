@mld @matrix @testbot
Feature: Basic Distance Matrix
# note that results are travel time, specified in 1/10th of seconds
# since testbot uses a default speed of 100m/10s, the result matches
# the number of meters as long as the way type is the default 'primary'

    Background:
        Given the profile "testbot"
        And the partition extra arguments "--small-component-size 1 --max-cell-sizes 2,4,8,16"

    Scenario: Testbot - mld Travel matrix of minimal network only
        Given the node map
            """
            a b
            """

        And the ways
            | nodes |
            | ab    |

        When I request a travel distance matrix I should get
            |   | a   | b   |
            | a | 0   | 9.9 |
            | b | 9.9 | 0   |

    Scenario: Testbot - mld Travel matrix of minimal network with excludes
        Given the query options
            | exclude  | toll        |

        Given the node map
            """
            a b
            c d
            """

        And the ways
            | nodes | highway  | toll | #                                        |
            | ab    | motorway |      | not drivable for exclude=motorway        |
            | cd    | primary  |      | always drivable                          |
            | ac    | highway  | yes  | not drivable for exclude=toll and exclude=motorway,toll |
            | bd    | highway  | yes  | not drivable for exclude=toll |

        When I request a travel distance matrix I should get
            |   |  a        |  b        |  c       |  d       |
            | a |  0        |  9.997721 |          |          |
            | b | 9.997721  |  0        |          |          |
            | c |           |           | 0        | 9.997724 |
            | d |           |           | 9.997724 | 0        |

    Scenario: Testbot - mld Travel matrix of minimal network with different exclude
        Given the query options
            | exclude  | motorway  |

        Given the node map
            """
            a b
            c d
            """

        And the ways
            | nodes | highway  | toll | #                                        |
            | ab    | motorway |      | not drivable for exclude=motorway        |
            | cd    | primary  |      | always drivable                          |
            | ac    | highway  | yes  | not drivable for exclude=toll and exclude=motorway,toll |
            | bd    | highway  | yes  | not drivable for exclude=toll |

        When I request a travel distance matrix I should get
            |   |   a  |  b   |  c   |   d  |
            | a |   0  | 39.9 | 9.9  | 29.9 |
            | b | 39.9 |  0   | 29.9 | 9.9  |
            | c |  10  | 29.9 | 0    | 19.9 |
            | d | 29.9 | 9.9  | 19.9 |   0  |

    Scenario: Testbot - mld Travel matrix of minimal network with excludes combination
        Given the query options
            | exclude  | motorway,toll  |

        Given the node map
            """
            a b
            c d
            """

        And the ways
            | nodes | highway  | toll | #                                        |
            | ab    | motorway |      | not drivable for exclude=motorway        |
            | cd    | primary  |      | always drivable                          |
            | ac    | highway  | yes  | not drivable for exclude=toll and exclude=motorway,toll |
            | bd    | highway  | yes  | not drivable for exclude=toll |

        When I request a travel distance matrix I should get
            |   |  a       |  b       |  c        |  d       |
            | a |  0       | 9.997724 |  0        | 9.997724 |
            | b | 9.997724 |  0       | 9.997724  |  0       |
            | c |  0       | 9.997724 |  0        | 9.997724 |
            | d | 9.997724 |  0       |  9.997724 |  0       |

    Scenario: Testbot - mld Travel matrix with different way speeds
        Given the node map
            """
            a b c d
            """

        And the ways
            | nodes | highway   |
            | ab    | primary   |
            | bc    | secondary |
            | cd    | tertiary  |

        When I request a travel distance matrix I should get
            |   |  a |
            | a |  0 |
            | b | 10 |
            | c | 30 |
            | d | 60 |

    Scenario: Testbot - mld Travel matrix with fuzzy match
        Given the node map
            """
            a b
            """

        And the ways
            | nodes |
            | ab    |

        When I request a travel distance matrix I should get
            |   | a  | b  |
            | a | 0  | 10 |
            | b | 10 | 0  |

    Scenario: Testbot - mld Travel matrix of small grid
        Given the node map
            """
            a b c
            d e f
            """

        And the ways
            | nodes |
            | abc   |
            | def   |
            | ad    |
            | be    |
            | cf    |

        When I request a travel distance matrix I should get
            |   | a  | b  | e  | f  |
            | a | 0  | 10 | 20 | 30 |
            | b | 10 | 0  | 10 | 20 |
            | e | 20 | 10 | 0  | 10 |
            | f | 30 | 20 | 10 | 0  |

    Scenario: Testbot - mld Travel matrix of network with unroutable parts
        Given the node map
            """
            a b
            """

        And the ways
            | nodes | oneway |
            | ab    | yes    |

        When I request a travel distance matrix I should get
            |   | a | b  |
            | a | 0 | 10 |
            | b |   | 0  |

    Scenario: Testbot - mld Travel matrix of network with oneways
        Given the node map
            """
            x a b y
              d e
            """

        And the ways
            | nodes | oneway |
            | abeda | yes    |
            | xa    |        |
            | by    |        |


        When I request a travel distance matrix I should get
            |   | x  | y   | d  | e  |
            | x | 0  | 30  | 40 | 30 |
            | y | 50 | 0   | 30 | 20 |
            | d | 20 | 30  | 0  | 30 |
            | e | 30 | 40  | 10 | 0  |
