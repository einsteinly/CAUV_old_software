def get_all_variables(entity_type):
    fields = {}
    try:
        for variable in entity_type.all()[0]: # get the name/type of variables in the first member of the bag
            fields[variable] = type(entity_type.all()[0][variable])
    except IndexError:
        pass
    fields.update(entity_type.allowed_types)
    print fields
    return fields
