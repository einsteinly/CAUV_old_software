function result = apply_iir(filter_num, filter_denom, data)
    % Apply IIR filter to data
    %
    % Hugo Vincent, 19 Jan 2010

    % Built-in wrapper:
    result = filter(filter_num, filter_denom, data);
    
%     % Explicit implementation: FIXME this is FIR not IIR
%     result = zeros(size(data));
%     for i = 1:length(data)
%         for j = 1:length(filter_vec)
%             if i-j > 0
%                 result(i) = result(i) + data(i-j) * filter_vec(j);  
%             end
%         end
%     end
end