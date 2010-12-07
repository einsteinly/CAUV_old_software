known_pitz_fields = ('modified_time','created_time','frag','type','yaml_file_saved','uuid' )
def get_all_variables(entity_type):
    fields = {}
    try:
        for variable in entity_type.all()[0]: # get the name/type of variables in the first member of the bag
	    if not variable in known_pitz_fields:
                fields[variable] = type(entity_type.all()[0][variable])
    except IndexError:
        pass
    fields.update(entity_type.allowed_types)
    return fields
