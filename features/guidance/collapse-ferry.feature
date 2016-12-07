@routing  @guidance @collapsing
Feature: Collapse

    Background:
        Given the profile "car"
        Given a grid size of 20 meters

    Scenario: Collapse Steps While On Ferry
        Given the node map
            """
            j    a   c   b    k


                        d


                                    f
                                e
            """

        And the ways
            | nodes | highway | route | name |
            | jacbk | primary |       | land |
            | ad    |         | ferry | sea  |
            | bd    |         | ferry | sea  |
            | cd    |         | ferry | sea  |
            | de    |         | ferry | sea  |
            | ef    | primary |       | land |

        When I route I should get
            | waypoints | route              | turns                                      | modes                         |
            | f,j       | land,sea,land,land | depart,notification right,turn left,arrive | driving,ferry,driving,driving |

    Scenario: Collapse Steps While On Ferry
        Given the node map
            """
            j    a   c   b    k


                        d


                        g


                                    f
                                e
            """

        And the ways
            | nodes | highway | route | name |
            | jacbk | primary |       | land |
            | ad    |         | ferry | sea  |
            | bd    |         | ferry | sea  |
            | cd    |         | ferry | sea  |
            | dg    |         | ferry | sea  |
            | ge    |         | ferry | sea  |
            | ef    | primary |       | land |

        When I route I should get
            | waypoints | route              | turns                                      | modes                         |
            | f,j       | land,sea,land,land | depart,notification right,turn left,arrive | driving,ferry,driving,driving |

    Scenario: Switching Ferry in a Harbour
        Given the node map
            """
                          d
                          |
                          |
                          |
            e - a ~ ~ ~ ~ b
                          ~
                          ~
                          ~
                          c
                          |
                          f
            """

        And the ways
            | nodes | highway | route | name                |
            | ea    | primary |       | melee-island        |
            | ab    |         | ferry | melee-island-ferry  |
            | cf    | primary |       | monkey-island       |
            | bd    | primary |       | landmass            |
            | bc    | primary | ferry | monkey-island-ferry |

        When I route I should get
            | waypoints | route                                                                           | turns                                                                | modes                               |
            | e,f       | melee-island,melee-island-ferry,monkey-island-ferry,monkey-island,monkey-island | depart,notification straight,turn right,notification straight,arrive | driving,ferry,ferry,driving,driving |
