import jax

## Some configurations
jax.config.update("jax_enable_x64", True)# Enable 64-bit once, globally

def jit(fun=None, **kwargs):
    """Wrapper around jax.jit that sets backend=cpu by default."""
    kwargs.setdefault("backend", "cpu")
    if fun is None:
        # Called like @jit(...)
        return lambda f: jax.jit(f, **kwargs)
    else:
        # Called like @jit
        return jax.jit(fun, **kwargs)
