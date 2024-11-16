def wmae_loss():
    def loss(y_true, y_pred):
        ## Coefficient
        c = 5
        
        ## Normalization
        y_true_norm = (255 - y_true) / 255
        y_pred_norm = (255 - y_pred) / 255

        ## Delta
        delta = y_true_norm - y_pred_norm

        ## Weighted Absolute Error
        greater = K.cast(K.greater(delta, 0),"float32")
        less = K.cast(K.less(delta, 0),"float32")
        w = K.l2_normalize(greater*y_true_norm,axis=-1) * c + less
        loss = K.mean(K.abs(w*delta))
        
        return loss 
    return loss
