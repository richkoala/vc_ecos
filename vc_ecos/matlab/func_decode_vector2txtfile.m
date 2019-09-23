function data_byte_len = func_decode_vector2txtfile(fid,fid_config,format)

    dat = fscanf(fid,format,inf);
    for idx = 1:length(dat)-3
        if (format == '%d')
            fprintf(fid_config,'%d\n',dat(idx));
        else if (format == '%f')
            fprintf(fid_config,'%.16e\n',dat(idx));
            else
            disp('format is not int32 and double');
            end
        end
    end
    
    data_byte_len = (length(dat)-3)*8;
    
end