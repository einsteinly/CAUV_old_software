function add_extra(form_name){
total_forms_field = document.getElementById('id_'+form_name+'-TOTAL_FORMS');
total_forms_field.value = parseInt(total_forms_field.value) + 1;
document.order_filter_form.submit();
}
