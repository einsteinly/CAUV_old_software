def BoostMapToDict(boost_map):
    return dict([(key, value.value) for key, value in boost_map.items()])
