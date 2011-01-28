/*
Note on javascript annoyingness.....
any function has access to, and will overwrite if another value of the same name is used by that function, variables in the parent scope...
*/
function add_extra(form_name){
    var total_forms_field = document.getElementById('id_'+form_name+'-TOTAL_FORMS');
    total_forms_field.value = parseInt(total_forms_field.value) + 1;
    document.order_filter_form.submit();
    }   
function create_select(dictionary, def){
    var input = document.createElement('select');
    for (var x in filter_names){
        var option = document.createElement('option');
        option.value = filter_names[x];
        option.text = all_filters[filter_names[x]][0];
        input.options[input.options.length]=option;
        if(option.value==def){
            input.selectedIndex=x;
            }
        }
    return input
    }
function generate(type, value, previous){
    var parent_div = document.createElement('div');
    parent_div.className = 'filter';
    parent_div.filter_type = type;
    var filter = create_select(all_filters,type);
    filter.addEventListener('change',function stuff(){update(parent_div, filter)},false)
    switch(type){
        case 'va':
            //value
            parent_div.appendChild(filter);
            var input_field = document.createElement('input');
            input_field.value = value;
            parent_div.appendChild(input_field);
            break
        case 'ep':
            //entity property
            parent_div.appendChild(filter);
            var input_field = document.createElement('input');
            input_field.value = value;
            parent_div.appendChild(input_field);
            break
        case 'md':
            //match dicts are special
            parent_div.appendChild(filter);
            var input_field = document.createElement('input');
            input_field.value = value;
            parent_div.appendChild(input_field);
            break
        default:
            //filter
            var type_info = all_filters[type];
            //create values
            if(value.length){var indices = value.split(',');}
            else{var indices = []}
            var v_as_obj = [];
            for(var i in indices){
                v_as_obj[i]=previous[indices[i]];
                v_as_obj[i].parent=parent_div;
                }
            switch(type_info[1]){
                case 0:
                    parent_div.appendChild(filter);
                    break
                case 2:
                    while(v_as_obj.length<2){
                        var sub_obj = generate('-','',[]);
                        sub_obj.parent=parent_div
                        v_as_obj[v_as_obj.length]=sub_obj;
                        }
                    parent_div.appendChild(v_as_obj[0]);
                    parent_div.appendChild(filter);
                    parent_div.appendChild(v_as_obj[1]);
                    break
                case -1:
                    parent_div.appendChild(filter);
                    for(var i=0;i<v_as_obj.length;i++){
                        parent_div.appendChild(v_as_obj[i]);
                        }
                    var sub_obj = generate('-','',[]);
                    sub_obj.parent=parent_div
                    parent_div.appendChild(sub_obj);
                    break
                default:
                    parent_div.appendChild(filter);
                    for(var i=0;i<Math.min(v_as_obj.length,type_info[1]);i++){
                        parent_div.appendChild(v_as_obj[i]);
                        }
                    if(v_as_obj.length<type_info[1]){
                        for(var f=0;f<type_info[1]-Math.min(v_as_obj.length);f++){
                            var sub_obj = generate('-','',[]);
                            sub_obj.parent=parent_div;
                            parent_div.appendChild(sub_obj);
                            }
                        }
                    break
                }
        }
    return parent_div
    }
function update(obj, filter){
    //first, rescue the values
    var values = [];
    var indices = []
    for(var node in obj.childNodes){
        if(obj.childNodes[node].tagName=='DIV'){
            if(obj.childNodes[node].filter_type != '-'){
                values[values.length] = obj.childNodes[node]
                indices[indices.length] = indices.length
                }
            }
        }
    //then destroy the node and start again
    var new_obj = generate(filter_names[filter.selectedIndex],indices.join(','), values)
    new_obj.parent = obj.parent
    obj.parent.replaceChild(new_obj,obj)
    }
function get_str(obj, string, counter){
    //if basic (ie stores a value rather than ref to another filter
    if(obj.filter_type=='va' || obj.filter_type=='ep' || obj.filter_type=='md'){
        string += '&f='+obj.filter_type+'&v='+obj.childNodes[1].value;
        counter += 1;
        return [string, counter]
        }
    //else
    var var_pos=[];
    // get strings from child nodes
    for(var node in obj.childNodes){
        if(obj.childNodes[node].tagName=='DIV'){
            //we dont want to send back these (as thatll cause an error)
            if(obj.childNodes[node].filter_type != '-'){
                result = get_str(obj.childNodes[node], string, counter)
                string = result[0]
                counter = result[1]
                var_pos[var_pos.length]=counter-1
                }
            }
        }
    string += '&f='+obj.filter_type+'&v='+var_pos.join(',')
    counter += 1
    return [string, counter]
    }
function filter(){
    var string='?';
    var counter=0;
    var result = get_str(document.getElementById('filterSpace').childNodes[0], string, counter)
    window.location=result[0]
    }
function load(){
    var previous=[];
    for(var e=0;e<filters.length;e++){
        previous[e]=generate(filters[e],values[e],previous);
        }
    if(previous.length > 0){cur = previous[filters.length-1]}
    else{cur=generate('-','',[])}
    cur.parent = document.getElementById('filterSpace');
    document.getElementById('filterSpace').appendChild(cur);
    }
//display stuff
function show(bag,counter){
    var child = document.getElementById(bag+'_'+counter);
    var parent_obj = document.getElementById(bag+'_dbox');
    if(parent_obj.hasChildNodes()){
        for(child2 in parent_obj.childNodes){
                if(parent_obj.childNodes[child2].style){parent_obj.childNodes[child2].style.display='none';}
            } 
        }
    if(child.style){child.style.display='block';}
    }
function scopepreserver(a,b) {
  return function () {
    show(a,b);
  };
}
function load_display_handlers(){
    var related_bags = document.getElementById('related_bags');
    for(cont in related_bags.childNodes){
        if(related_bags.childNodes[cont].tagName=='DIV'){
            var bag = related_bags.childNodes[cont].children[0];
            var id = bag.id;
            var dbox = document.getElementById(id+'_dbox');
            var ul;
            for(var child in bag.childNodes){
                if(bag.childNodes[child].tagName == 'UL'){
                    ul = bag.childNodes[child];
                    }
                }
            for(var li in ul.childNodes){
                if(ul.childNodes[li].tagName=='LI'){
                    var counter = ul.childNodes[li].id;
                    var new_function = scopepreserver(id, counter);
                    hidden_text = document.getElementById(id+'_'+counter);
                    dbox.appendChild(hidden_text);
                    if(hidden_text.style){hidden_text.style.display='none';}
                    //Grrrr IE :(
                    if(ul.childNodes[li].addEventListener){ul.childNodes[li].addEventListener('mouseover',new_function,false);}
                    else{ul.childNodes[li].attachEvent('onmouseover',new_function);}
                    }
                }
            }
        }
    }