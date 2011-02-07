/*
Note on javascript annoyingness.....
any function has access to, and will overwrite if another value of the same name is used by that function, variables in the parent scope...
*/
function create_select_from_dict(dictionary, def){
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
function generate_filter(type, value, previous){
    var parent_div = document.createElement('div');
    parent_div.className = 'filter';
    parent_div.filter_type = type;
    var filter = create_select_from_dict(all_filters,type);
    filter.addEventListener('change',function stuff(){update_filter(parent_div, filter)},false)
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
                        var sub_obj = generate_filter('-','',[]);
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
                    var sub_obj = generate_filter('-','',[]);
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
                            var sub_obj = generate_filter('-','',[]);
                            sub_obj.parent=parent_div;
                            parent_div.appendChild(sub_obj);
                            }
                        }
                    break
                }
        }
    return parent_div
    }
function update_filter(obj, filter){
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
    var new_obj = generate_filter(filter_names[filter.selectedIndex],indices.join(','), values)
    new_obj.parent = obj.parent
    obj.parent.replaceChild(new_obj,obj)
    }
function get_fltr_str(obj, string, counter){
    //if basic (ie stores a value rather than ref to another filter
    if(obj.filter_type=='va' || obj.filter_type=='ep' || obj.filter_type=='md'){
        string += '&f='+obj.filter_type+'&v='+obj.childNodes[1].value;
        counter += 1;
        return [string, counter]
        }
    else if(obj.filter_type=='-'){
        return [string, counter]
        }
    //else
    var var_pos=[];
    // get strings from child nodes
    for(var node in obj.childNodes){
        if(obj.childNodes[node].tagName=='DIV'){
            //we dont want to send back these (as thatll cause an error)
            if(obj.childNodes[node].filter_type != '-'){
                result = get_fltr_str(obj.childNodes[node], string, counter)
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
function get_odr_str(){
    orderSpace = document.getElementById('orderSpace');
    ostring = '';
    for(var x=0;x<orderSpace.childNodes.length;x++){
        if(orderSpace.childNodes[x].childNodes[0].options[orderSpace.childNodes[x].childNodes[0].selectedIndex].value != '-'){
            ostring += '&o='+orderSpace.childNodes[x].childNodes[0].options[orderSpace.childNodes[x].childNodes[0].selectedIndex].value + '&r=';
            if(orderSpace.childNodes[x].childNodes[1].checked){ostring += '1'}
            }
        }
    return ostring
    }
function filter(){
    var string='?';
    var counter=0;
    var result_filter = get_fltr_str(document.getElementById('filterSpace').childNodes[0], string, counter)
    var result_order = get_odr_str()
    window.location=result_filter[0]+result_order
    }
function update_order(){
    //declares function so it can be used before being defined
    }
function create_order(defE, defR){
    defE = typeof(defE) != 'undefined' ? defE : '-';
    defR = typeof(defR) != 'undefined' ? defR : false;
    var order_obj = document.createElement('div');
    var select = document.createElement('select');
    var reverse = document.createElement('input');
    var txt = document.createTextNode("R");
    order_obj.className = 'order_obj';
    order_obj.appendChild(select);
    order_obj.appendChild(reverse);
    order_obj.appendChild(txt);
    reverse.type = 'checkbox'
    select.addEventListener('change', update_order, false);
    for (var key in order_keys){
        var option = document.createElement('option');
        option.value = order_keys[key];
        option.text = order_keys[key];
        select.options[select.options.length]=option;
        if(option.value==defE){
            select.selectedIndex=key;
            }
        }
    if(defR == "1"){reverse.checked = true;}
    return order_obj
    }
function update_order(){
    //get the order space
    orderSpace = document.getElementById('orderSpace');
    //delete those elements that are empty
    for(var x=0;x<orderSpace.childNodes.length;x++){
        order_obj = orderSpace.childNodes[x];
        if(order_obj.childNodes[0].selectedIndex==0){
            orderSpace.removeChild(order_obj);
            }
        }
    //add an empt order at the end
    orderSpace.appendChild(create_order());
    }
function bag_load(){
    var previous=[];
    for(var e=0;e<filters.length;e++){
        previous[e]=generate_filter(filters[e],values[e],previous);
        }
    if(previous.length > 0){cur = previous[filters.length-1]}
    else{cur=generate_filter('-','',[])}
    cur.parent = document.getElementById('filterSpace');
    document.getElementById('filterSpace').appendChild(cur);
    order_keys.unshift('-');
    orderSpace = document.getElementById('orderSpace');
    for(var j in order){
        orderSpace.appendChild(create_order(order[j], reverse[j]))
        }
    orderSpace.appendChild(create_order())
    }
//display stuff
function show_bag(bag,counter){
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
    show_bag(a,b);
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
//shorcut form stuff
function show_shortcut_form(){
document.getElementById('shortcut_form_div').style = 'display:block';
document.getElementById('add_shortcut1').style = 'display:none';
}
function hide_shortcut_form(){
document.getElementById('shortcut_form_div').style = 'display:none';
document.getElementById('add_shortcut1').style = 'display:inline';
}
function submit_shortcut_form(){
document.getElementById('shortcut_form').submit();
}
function show_bag_form(){
document.getElementById('bag_form_div').style = 'display:block';
document.getElementById('add_bag1').style = 'display:none';
}
function hide_bag_form(){
document.getElementById('bag_form_div').style = 'display:none';
document.getElementById('add_bag1').style = 'display:inline';
}
function submit_bag_form(){
document.getElementById('bag_form').submit();
}
//generic load